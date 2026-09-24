#include "camera_hunt.h"

#include <windows.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../common/config.h"
#include "../common/log.h"
#include "hw_watch.h"

namespace edvr {

namespace {

enum class Stage { Off, Steadying, Scanning, Tracking, Watching, Done };
Stage g_stage = Stage::Off;
bool g_wanted = false;
uintptr_t g_base = 0;
uint64_t g_imageSize = 0;

// The camera as the game wrote it: right, up, forward (float4 rows).
float g_last[12] = {};
uint32_t g_steady = 0;
float g_pattern[12] = {};  // the rows the scan looks for (static: never in the scanned heap)

// Scan results. kind: 4 float rows, 8 double rows. The first flight kept the
// first 64 of 1392 hits by address: all thread stacks and the render
// thread's per-view b1 staging (0x1500 apart, b1's size) -- the render
// side's copies. Now every hit is kept, stacks are skipped (an allocation
// with a guard page), and a hit follows the camera if it holds any of the
// last four frames' forward rows: the game thread's camera is a frame or two
// AHEAD of what the render thread writes into b1.
struct Candidate {
    uintptr_t addr;  // the forward row's first element
    uint8_t kind;
    uint8_t rows;    // 1 packed rows, 2 padded rows before it: a rotation by rows
    bool live, array;
    uint32_t matched, compared;
    float ring[4][3];
};
constexpr LONG kMaxCandidates = 16384;
Candidate* g_cand = nullptr;
volatile LONG g_candCount = 0, g_scanDone = 0;
volatile LONG64 g_scannedBytes = 0;
volatile LONG g_scannedRegions = 0, g_skippedStacks = 0;
uint8_t* g_scanBuf = nullptr;
constexpr SIZE_T kChunk = 1u << 20;
ULONGLONG g_scanStartMs = 0, g_stageMs = 0;
uint32_t g_trackFrames = 0, g_movedFrames = 0;
bool g_moveHinted = false;
float g_recent[4][3] = {};  // the last four frames' forward rows (the render side)

// The write-watch: three candidates a window, doubles first.
int g_order[kMaxCandidates];
int g_orderCount = 0, g_orderNext = 0, g_windows = 0;
constexpr int kMaxWindows = 12;
int g_watchIdx[3] = {-1, -1, -1};
PVOID g_veh = nullptr;
volatile LONG g_vehArmed = 0;
constexpr int kMaxWriters = 8, kMaxFrames = 12;
struct Writer {
    volatile LONG count;
    uint32_t rip;
    uint32_t frames[kMaxFrames];
    uint32_t n;
};
Writer g_writers[3][kMaxWriters];
volatile LONG g_writerCount[3] = {};
volatile LONG g_writerOverflow[3] = {};

constexpr double kTol = 2e-6;

LONG Count() { return g_candCount < kMaxCandidates ? g_candCount : kMaxCandidates; }

bool Near3(const float* v, const float* f) {
    return std::fabs(v[0] - f[0]) <= kTol && std::fabs(v[1] - f[1]) <= kTol && std::fabs(v[2] - f[2]) <= kTol;
}
bool Near3d(const double* v, const float* f) {
    return std::fabs(v[0] - f[0]) <= kTol && std::fabs(v[1] - f[1]) <= kTol && std::fabs(v[2] - f[2]) <= kTol;
}

bool ReadOwn(uintptr_t at, void* out, SIZE_T bytes) {
    SIZE_T got = 0;
    return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<const void*>(at), out, bytes, &got) &&
           got == bytes;
}

// A candidate's forward row as floats, read in place (a freed page faults
// into the handler, not the game).
__declspec(noinline) bool ReadForward(const Candidate& c, float out[3]) noexcept {
    __try {
        if (c.kind == 4) {
            const volatile float* v = reinterpret_cast<const volatile float*>(c.addr);
            out[0] = v[0];
            out[1] = v[1];
            out[2] = v[2];
        } else {
            const volatile double* v = reinterpret_cast<const volatile double*>(c.addr);
            out[0] = static_cast<float>(v[0]);
            out[1] = static_cast<float>(v[1]);
            out[2] = static_cast<float>(v[2]);
        }
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void AddCandidate(uintptr_t addr, uint8_t kind, uint8_t rows) {
    const LONG i = InterlockedIncrement(&g_candCount) - 1;
    if (i >= kMaxCandidates) return;
    Candidate& c = g_cand[i];
    memset(&c, 0, sizeof(c));
    c.addr = addr;
    c.kind = kind;
    c.rows = rows;
}

void SearchChunk(const uint8_t* buf, SIZE_T n, uintptr_t at) {
    const float* fwd = g_pattern + 8;
    for (SIZE_T o = 0; o + 12 <= n; o += 4) {
        const float* v = reinterpret_cast<const float*>(buf + o);
        if (std::fabs(v[0] - fwd[0]) > kTol || !Near3(v, fwd)) continue;
        uint8_t rows = 0;
        for (SIZE_T stride : {SIZE_T(12), SIZE_T(16)}) {
            if (o >= 2 * stride && Near3(reinterpret_cast<const float*>(buf + o - 2 * stride), g_pattern) &&
                Near3(reinterpret_cast<const float*>(buf + o - stride), g_pattern + 4))
                rows = stride == 12 ? 1 : 2;
        }
        AddCandidate(at + o, 4, rows);
    }
    for (SIZE_T o = 0; o + 24 <= n; o += 8) {
        const double* v = reinterpret_cast<const double*>(buf + o);
        if (std::fabs(v[0] - fwd[0]) > kTol || !Near3d(v, fwd)) continue;
        uint8_t rows = 0;
        for (SIZE_T stride : {SIZE_T(24), SIZE_T(32)}) {
            if (o >= 2 * stride && Near3d(reinterpret_cast<const double*>(buf + o - 2 * stride), g_pattern) &&
                Near3d(reinterpret_cast<const double*>(buf + o - stride), g_pattern + 4))
                rows = stride == 24 ? 1 : 2;
        }
        AddCandidate(at + o, 8, rows);
    }
}

// A thread's stack: its reservation carries a guard page below the
// committed part.
bool IsStack(uintptr_t allocationBase) {
    MEMORY_BASIC_INFORMATION m{};
    uintptr_t p = allocationBase;
    for (int i = 0; i < 64 && VirtualQuery(reinterpret_cast<const void*>(p), &m, sizeof(m)) == sizeof(m); ++i) {
        if (reinterpret_cast<uintptr_t>(m.AllocationBase) != allocationBase) break;
        if (m.State == MEM_COMMIT && (m.Protect & PAGE_GUARD)) return true;
        p = reinterpret_cast<uintptr_t>(m.BaseAddress) + m.RegionSize;
    }
    return false;
}

DWORD WINAPI ScanProc(LPVOID) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
    MEMORY_BASIC_INFORMATION mbi{};
    uintptr_t p = 0x10000;
    const uintptr_t bufLo = reinterpret_cast<uintptr_t>(g_scanBuf), bufHi = bufLo + kChunk + 64;
    const uintptr_t candLo = reinterpret_cast<uintptr_t>(g_cand), candHi = candLo + kMaxCandidates * sizeof(Candidate);
    uintptr_t lastAlloc = 0;
    bool lastStack = false;
    while (p < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<const void*>(p), &mbi, sizeof(mbi)) == sizeof(mbi)) {
        const uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
        const uintptr_t end = base + mbi.RegionSize;
        const bool rw = (mbi.Protect & 0xFF) == PAGE_READWRITE && !(mbi.Protect & (PAGE_GUARD | PAGE_NOCACHE));
        const bool ours = (base < bufHi && end > bufLo) || (base < candHi && end > candLo);
        if (mbi.State == MEM_COMMIT && mbi.Type == MEM_PRIVATE && rw && !ours) {
            const uintptr_t alloc = reinterpret_cast<uintptr_t>(mbi.AllocationBase);
            if (alloc != lastAlloc) {
                lastAlloc = alloc;
                lastStack = IsStack(alloc);
                if (lastStack) InterlockedIncrement(&g_skippedStacks);
            }
            if (!lastStack) {
                InterlockedIncrement(&g_scannedRegions);
                for (uintptr_t at = base; at < end; at += kChunk) {
                    const SIZE_T n = static_cast<SIZE_T>((end - at) < kChunk + 24 ? (end - at) : kChunk + 24);
                    if (!ReadOwn(at, g_scanBuf, n)) continue;
                    SearchChunk(g_scanBuf, n, at);
                    InterlockedAdd64(&g_scannedBytes, static_cast<LONG64>(n < kChunk ? n : kChunk));
                }
            }
        }
        if (end <= p) break;
        p = end;
    }
    InterlockedExchange(&g_scanDone, 1);
    return 0;
}

// A candidate's neighbourhood, 32 values before and after it, in its kind.
void Dump(const Candidate& c, int idx, const char* when) {
    char line[1024];
    if (c.kind == 4) {
        float v[64];
        if (!ReadOwn(c.addr - 128, v, sizeof(v))) return;
        for (int r = 0; r < 8; ++r) {
            int n = snprintf(line, sizeof(line), "camera hunt: %s hit %d %+5d:", when, idx, (r * 8 - 32) * 4);
            for (int k = 0; k < 8 && n < static_cast<int>(sizeof(line)) - 20; ++k)
                n += snprintf(line + n, sizeof(line) - n, " %.6g", v[r * 8 + k]);
            Log::get().note("%s", line);
        }
    } else {
        double v[32];
        if (!ReadOwn(c.addr - 128, v, sizeof(v))) return;
        for (int r = 0; r < 4; ++r) {
            int n = snprintf(line, sizeof(line), "camera hunt: %s hit %d %+5d (doubles):", when, idx, (r * 8 - 16) * 8);
            for (int k = 0; k < 8 && n < static_cast<int>(sizeof(line)) - 24; ++k)
                n += snprintf(line + n, sizeof(line) - n, " %.9g", v[r * 8 + k]);
            Log::get().note("%s", line);
        }
    }
}

LONG CALLBACK HuntVeh(EXCEPTION_POINTERS* ep) {
    if (!ep || !ep->ExceptionRecord || !ep->ContextRecord || !g_vehArmed) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    const DWORD64 dr6 = ep->ContextRecord->Dr6;
    if (!(dr6 & 0xE)) return EXCEPTION_CONTINUE_SEARCH;
    ep->ContextRecord->Dr6 = dr6 & ~DWORD64(0xE);
    const uintptr_t rip = reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress);
    const uint32_t rva = (rip >= g_base && rip < g_base + g_imageSize) ? static_cast<uint32_t>(rip - g_base)
                                                                        : 0xFFFFFFFFu;
    for (int s = 0; s < 3; ++s) {
        if (!(dr6 & (DWORD64(2) << s))) continue;
        const LONG have = g_writerCount[s] < kMaxWriters ? g_writerCount[s] : kMaxWriters;
        bool found = false;
        for (LONG i = 0; i < have && !found; ++i)
            if (g_writers[s][i].rip == rva) {
                InterlockedIncrement(&g_writers[s][i].count);
                found = true;
            }
        if (found) continue;
        const LONG i = InterlockedIncrement(&g_writerCount[s]) - 1;
        if (i >= kMaxWriters) {
            InterlockedIncrement(&g_writerOverflow[s]);
            continue;
        }
        Writer& w = g_writers[s][i];
        w.n = unwindGameStack(*ep->ContextRecord, g_base, g_imageSize, w.frames, kMaxFrames);
        w.count = 1;
        MemoryBarrier();
        w.rip = rva;
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

void StartScan() {
    if (!g_scanBuf)
        g_scanBuf = static_cast<uint8_t*>(VirtualAlloc(nullptr, kChunk + 64, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!g_cand)
        g_cand = static_cast<Candidate*>(
            VirtualAlloc(nullptr, kMaxCandidates * sizeof(Candidate), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (!g_scanBuf || !g_cand) {
        Log::get().note("camera hunt: no memory for the scan; stopped.");
        g_stage = Stage::Done;
        return;
    }
    memcpy(g_pattern, g_last, sizeof(g_pattern));
    g_candCount = 0;
    g_scanDone = 0;
    g_scannedBytes = 0;
    g_scannedRegions = g_skippedStacks = 0;
    HANDLE h = CreateThread(nullptr, 0, &ScanProc, nullptr, 0, nullptr);
    if (!h) {
        g_stage = Stage::Done;
        return;
    }
    CloseHandle(h);
    g_scanStartMs = GetTickCount64();
    g_stage = Stage::Scanning;
    Log::get().note("camera hunt: the camera is still; searching memory for its forward row (%.6f %.6f %.6f), as "
                    "floats and as doubles.",
                    g_pattern[8], g_pattern[9], g_pattern[10]);
}

void LogScan() {
    const LONG n = Count();
    LONG floats = 0, doubles = 0, matrices = 0;
    for (LONG i = 0; i < n; ++i) {
        (g_cand[i].kind == 4 ? floats : doubles)++;
        if (g_cand[i].rows) ++matrices;
    }
    Log::get().note("camera hunt: scan done in %llu ms: %ld regions (%ld stacks skipped), %.0f MB, %ld hits (%ld "
                    "floats, %ld doubles, %ld with the right and up rows before them)%s. Now turn with the stick "
                    "(not the head) for a few seconds.",
                    static_cast<unsigned long long>(GetTickCount64() - g_scanStartMs), static_cast<long>(g_scannedRegions),
                    static_cast<long>(g_skippedStacks), static_cast<double>(g_scannedBytes) / (1024.0 * 1024.0),
                    static_cast<long>(g_candCount), static_cast<long>(floats), static_cast<long>(doubles),
                    static_cast<long>(matrices), g_candCount > kMaxCandidates ? " (the first 16384 kept)" : "");
}

void Track(const float* raw) {
    for (int k = 3; k > 0; --k) memcpy(g_recent[k], g_recent[k - 1], sizeof(g_recent[0]));
    memcpy(g_recent[0], raw + 8, sizeof(g_recent[0]));
    ++g_trackFrames;
    const LONG n = Count();
    const bool moved = !Near3(raw + 8, g_pattern + 8);
    if (moved) ++g_movedFrames;
    for (LONG i = 0; i < n; ++i) {
        Candidate& c = g_cand[i];
        for (int k = 3; k > 0; --k) memcpy(c.ring[k], c.ring[k - 1], sizeof(c.ring[0]));
        if (!ReadForward(c, c.ring[0])) c.ring[0][0] = c.ring[0][1] = c.ring[0][2] = 1e30f;
        if (!moved || g_trackFrames < 5) continue;
        // This frame's render-side row, held now or in the last frames by
        // the candidate (ahead) or the other way round (behind).
        ++c.compared;
        bool hit = false;
        for (int k = 0; k < 4 && !hit; ++k) hit = Near3(c.ring[k], g_recent[0]) || Near3(c.ring[0], g_recent[k]);
        if (hit) ++c.matched;
    }
}

// Followers, doubles first, then floats; the per-view arrays (another
// follower 0x1500 away: b1's staging) last.
void Order() {
    const LONG n = Count();
    g_orderCount = 0;
    int live = 0, arrays = 0;
    for (LONG i = 0; i < n; ++i) {
        Candidate& c = g_cand[i];
        c.live = c.compared >= 60 && c.matched * 10 >= c.compared * 8;
        if (c.live) ++live;
    }
    for (LONG i = 0; i < n; ++i) {
        if (!g_cand[i].live) continue;
        for (LONG j = 0; j < n && !g_cand[i].array; ++j) {
            if (i == j || !g_cand[j].live) continue;
            const uintptr_t d = g_cand[i].addr > g_cand[j].addr ? g_cand[i].addr - g_cand[j].addr
                                                                : g_cand[j].addr - g_cand[i].addr;
            if (d == 0x1500) g_cand[i].array = true;
        }
        if (g_cand[i].array) ++arrays;
    }
    for (int pass = 0; pass < 3; ++pass)
        for (LONG i = 0; i < n; ++i) {
            const Candidate& c = g_cand[i];
            if (!c.live) continue;
            const int want = c.array ? 2 : (c.kind == 8 ? 0 : 1);
            if (want == pass) g_order[g_orderCount++] = static_cast<int>(i);
        }
    Log::get().note("camera hunt: %d hits follow the camera (%d in per-view arrays) over %u turned frames.", live,
                    arrays, g_movedFrames);
    for (int k = 0; k < g_orderCount && k < 120; ++k) {
        const Candidate& c = g_cand[g_order[k]];
        MEMORY_BASIC_INFORMATION mbi{};
        VirtualQuery(reinterpret_cast<const void*>(c.addr), &mbi, sizeof(mbi));
        Log::get().note("camera hunt: follower hit %d at %p (%s%s%s), allocation %p, %u of %u.", g_order[k],
                        reinterpret_cast<void*>(c.addr), c.kind == 4 ? "floats" : "doubles",
                        c.rows == 1 ? ", packed rows" : (c.rows == 2 ? ", padded rows" : ""),
                        c.array ? ", per-view array" : "", mbi.AllocationBase, c.matched, c.compared);
    }
}

bool ArmNext() {
    if (g_windows >= kMaxWindows) return false;
    uintptr_t addr[3] = {};
    uint8_t len[3] = {};
    int got = 0;
    while (got < 3 && g_orderNext < g_orderCount) {
        const int i = g_order[g_orderNext++];
        g_watchIdx[got] = i;
        addr[got] = g_cand[i].addr;
        len[got] = g_cand[i].kind;
        ++got;
    }
    for (int s = got; s < 3; ++s) g_watchIdx[s] = -1;
    if (!got) return false;
    ++g_windows;
    for (int s = 0; s < 3; ++s) {
        g_writerCount[s] = 0;
        g_writerOverflow[s] = 0;
        for (auto& w : g_writers[s]) w = Writer{};
    }
    for (int s = 0; s < got; ++s) Dump(g_cand[g_watchIdx[s]], g_watchIdx[s], "before");
    if (!g_veh) g_veh = AddVectoredExceptionHandler(1, &HuntVeh);
    InterlockedExchange(&g_vehArmed, 1);
    const int threads = hwWatchArm(addr, len);
    Log::get().note("camera hunt: watching writes to hits %d %d %d on %d threads.", g_watchIdx[0], g_watchIdx[1],
                    g_watchIdx[2], threads);
    g_stageMs = GetTickCount64();
    return true;
}

void EndWindow() {
    hwWatchDisarm();
    InterlockedExchange(&g_vehArmed, 0);
    for (int s = 0; s < 3; ++s) {
        if (g_watchIdx[s] < 0) continue;
        const Candidate& c = g_cand[g_watchIdx[s]];
        const LONG have = g_writerCount[s] < kMaxWriters ? g_writerCount[s] : kMaxWriters;
        if (!have)
            Log::get().note("camera hunt: hit %d (%p): no writes in the window.", g_watchIdx[s],
                            reinterpret_cast<void*>(c.addr));
        for (LONG i = 0; i < have; ++i) {
            const Writer& w = g_writers[s][i];
            char frames[300];
            int k = 0;
            frames[0] = 0;
            for (uint32_t f = 0; f < w.n && k < static_cast<int>(sizeof(frames)) - 16; ++f)
                k += snprintf(frames + k, sizeof(frames) - k, " +0x%X", w.frames[f]);
            Log::get().note("camera hunt: hit %d (%p) written %ld times at +0x%X; stack:%s", g_watchIdx[s],
                            reinterpret_cast<void*>(c.addr), static_cast<long>(w.count), w.rip, frames);
        }
        if (g_writerOverflow[s])
            Log::get().note("camera hunt: hit %d: %ld writes by more writers than kept.", g_watchIdx[s],
                            static_cast<long>(g_writerOverflow[s]));
        Dump(c, g_watchIdx[s], "after");
    }
}

}  // namespace

void cameraHuntConfigure(Config& cfg) {
    const bool want = cfg.getBool("experimental.onfoot_camera_hunt", false);
    if (want && !g_wanted) {
        if (!g_base) {
            g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
            const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
            const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew);
            g_imageSize = nt->OptionalHeader.SizeOfImage;
        }
        if (g_stage == Stage::Off || g_stage == Stage::Done) {
            g_stage = Stage::Steadying;
            g_steady = 0;
            Log::get().note("camera hunt: ON. On foot, stand still for two seconds (no stick, head anywhere), then "
                            "turn with the stick when the log asks.");
        }
    }
    if (!want && g_wanted) {
        if (g_stage == Stage::Watching) EndWindow();
        if (g_stage != Stage::Scanning) g_stage = Stage::Off;
        Log::get().note("camera hunt: off.");
    }
    g_wanted = want;
}

bool cameraHuntWatching() { return g_stage == Stage::Watching; }

void cameraHuntFrame(const float* raw12) {
    if (g_stage == Stage::Off || g_stage == Stage::Done) return;
    const ULONGLONG now = GetTickCount64();
    switch (g_stage) {
        case Stage::Steadying:
            if (!raw12 || !g_wanted) return;
            if (memcmp(raw12, g_last, sizeof(g_last)) == 0) {
                if (++g_steady == 120) StartScan();
            } else {
                g_steady = 0;
            }
            memcpy(g_last, raw12, sizeof(g_last));
            return;
        case Stage::Scanning:
            if (!g_scanDone) return;
            LogScan();
            if (!g_candCount) {
                Log::get().note("camera hunt: nothing found; standing still again restarts it.");
                g_stage = g_wanted ? Stage::Steadying : Stage::Off;
                g_steady = 0;
                return;
            }
            g_stage = Stage::Tracking;
            g_trackFrames = g_movedFrames = 0;
            g_moveHinted = false;
            g_stageMs = now;
            return;
        case Stage::Tracking: {
            if (!raw12) return;
            Track(raw12);
            if (!g_moveHinted && now - g_stageMs > 20000 && g_movedFrames < 10) {
                g_moveHinted = true;
                Log::get().note("camera hunt: waiting for the camera to turn -- use the stick.");
            }
            if (g_movedFrames < 300) return;
            Order();
            if (!g_orderCount) {
                Log::get().note("camera hunt: no hit follows the camera; done.");
                g_stage = Stage::Done;
                return;
            }
            g_orderNext = g_windows = 0;
            g_stage = ArmNext() ? Stage::Watching : Stage::Done;
            return;
        }
        case Stage::Watching:
            if (now - g_stageMs < 3000) {
                if (now - g_stageMs > 1000 && now - g_stageMs < 1100) hwWatchSweep();
                return;
            }
            EndWindow();
            if (!ArmNext()) {
                g_stage = Stage::Done;
                Log::get().note("camera hunt: done (%d windows).", g_windows);
            }
            return;
        default:
            return;
    }
}

}  // namespace edvr
