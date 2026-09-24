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

// Scan results. kind: 4 float rows, 8 double rows.
struct Candidate {
    uintptr_t addr;  // the forward row's first element
    uint8_t kind;
    uint8_t rows;    // bit0 right row before it, bit1 up row before it (same layout)
    uint32_t matched, compared;
    bool live;
};
constexpr int kMaxCandidates = 64;
Candidate g_cand[kMaxCandidates];
volatile LONG g_candCount = 0, g_scanDone = 0;
volatile LONG64 g_scannedBytes = 0;
volatile LONG g_scannedRegions = 0;
uint8_t* g_scanBuf = nullptr;
constexpr SIZE_T kChunk = 1u << 20;
ULONGLONG g_scanStartMs = 0, g_stageMs = 0;
uint32_t g_trackFrames = 0, g_movedFrames = 0;
bool g_moveHinted = false;

// The write-watch: three candidates a window.
int g_watchNext = 0;
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

void AddCandidate(uintptr_t addr, uint8_t kind, uint8_t rows) {
    const LONG i = InterlockedIncrement(&g_candCount) - 1;
    if (i >= kMaxCandidates) return;
    g_cand[i] = {addr, kind, rows, 0, 0, false};
}

// Searches one chunk read from [at, at + n) for the forward row, as floats
// (4-aligned) and as doubles (8-aligned), and notes whether the right and up
// rows sit before it in the same layout (a rotation matrix by rows).
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

// Every committed private read-write region, in chunks read through
// ReadProcessMemory (a region freed under the scan fails the read, not us).
DWORD WINAPI ScanProc(LPVOID) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
    MEMORY_BASIC_INFORMATION mbi{};
    uintptr_t p = 0x10000;
    const uintptr_t bufLo = reinterpret_cast<uintptr_t>(g_scanBuf), bufHi = bufLo + kChunk + 64;
    while (p < 0x7FFFFFFF0000ull && VirtualQuery(reinterpret_cast<const void*>(p), &mbi, sizeof(mbi)) == sizeof(mbi)) {
        const uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress);
        const uintptr_t end = base + mbi.RegionSize;
        const bool rw = (mbi.Protect & 0xFF) == PAGE_READWRITE && !(mbi.Protect & (PAGE_GUARD | PAGE_NOCACHE));
        if (mbi.State == MEM_COMMIT && mbi.Type == MEM_PRIVATE && rw && !(base < bufHi && end > bufLo)) {
            InterlockedIncrement(&g_scannedRegions);
            for (uintptr_t at = base; at < end; at += kChunk) {
                const SIZE_T n = static_cast<SIZE_T>((end - at) < kChunk + 24 ? (end - at) : kChunk + 24);
                if (!ReadOwn(at, g_scanBuf, n)) continue;
                SearchChunk(g_scanBuf, n, at);
                InterlockedAdd64(&g_scannedBytes, static_cast<LONG64>(n < kChunk ? n : kChunk));
            }
        }
        if (end <= p) break;
        p = end;
    }
    InterlockedExchange(&g_scanDone, 1);
    return 0;
}

// A candidate's neighbourhood, 32 values before and after, in its kind.
void Dump(const Candidate& c, const char* when) {
    char line[1024];
    if (c.kind == 4) {
        float v[64];
        if (!ReadOwn(c.addr - 128, v, sizeof(v))) return;
        for (int r = 0; r < 8; ++r) {
            int n = snprintf(line, sizeof(line), "camera hunt: %s %p %+5d:", when, reinterpret_cast<void*>(c.addr),
                             (r * 8 - 32) * 4);
            for (int k = 0; k < 8 && n < static_cast<int>(sizeof(line)) - 20; ++k)
                n += snprintf(line + n, sizeof(line) - n, " %.6g", v[r * 8 + k]);
            Log::get().note("%s", line);
        }
    } else {
        double v[32];
        if (!ReadOwn(c.addr - 128, v, sizeof(v))) return;
        for (int r = 0; r < 4; ++r) {
            int n = snprintf(line, sizeof(line), "camera hunt: %s %p %+5d (doubles):", when,
                             reinterpret_cast<void*>(c.addr), (r * 8 - 16) * 8);
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
    if (!g_scanBuf) {
        Log::get().note("camera hunt: no memory for the scan; stopped.");
        g_stage = Stage::Done;
        return;
    }
    memcpy(g_pattern, g_last, sizeof(g_pattern));
    g_candCount = 0;
    g_scanDone = 0;
    g_scannedBytes = 0;
    g_scannedRegions = 0;
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
    const LONG n = g_candCount < kMaxCandidates ? g_candCount : kMaxCandidates;
    Log::get().note("camera hunt: scan done in %llu ms: %ld regions, %.0f MB, %ld hits%s. Now turn with the stick "
                    "(not the head) for a few seconds.",
                    static_cast<unsigned long long>(GetTickCount64() - g_scanStartMs), static_cast<long>(g_scannedRegions),
                    static_cast<double>(g_scannedBytes) / (1024.0 * 1024.0), static_cast<long>(g_candCount),
                    g_candCount > kMaxCandidates ? " (the first 64 kept)" : "");
    for (LONG i = 0; i < n; ++i) {
        MEMORY_BASIC_INFORMATION mbi{};
        VirtualQuery(reinterpret_cast<const void*>(g_cand[i].addr), &mbi, sizeof(mbi));
        Log::get().note("camera hunt: hit %ld at %p (%s%s), allocation %p", static_cast<long>(i),
                        reinterpret_cast<void*>(g_cand[i].addr), g_cand[i].kind == 4 ? "floats" : "doubles",
                        g_cand[i].rows == 1 ? ", packed rows" : (g_cand[i].rows == 2 ? ", padded rows" : ""),
                        mbi.AllocationBase);
    }
}

void Track(const float* raw) {
    const LONG n = g_candCount < kMaxCandidates ? g_candCount : kMaxCandidates;
    const bool moved = !Near3(raw + 8, g_pattern + 8);
    if (!moved) return;
    ++g_movedFrames;
    for (LONG i = 0; i < n; ++i) {
        Candidate& c = g_cand[i];
        ++c.compared;
        if (c.kind == 4) {
            float v[3];
            if (ReadOwn(c.addr, v, sizeof(v)) && Near3(v, raw + 8)) ++c.matched;
        } else {
            double v[3];
            if (ReadOwn(c.addr, v, sizeof(v)) && Near3d(v, raw + 8)) ++c.matched;
        }
    }
}

// The next window's three candidates, armed; false when none are left.
bool ArmNext() {
    const LONG n = g_candCount < kMaxCandidates ? g_candCount : kMaxCandidates;
    uintptr_t addr[3] = {};
    uint8_t len[3] = {};
    int got = 0;
    while (got < 3 && g_watchNext < n) {
        const int i = g_watchNext++;
        if (!g_cand[i].live) continue;
        g_watchIdx[got] = i;
        addr[got] = g_cand[i].addr;
        len[got] = g_cand[i].kind;
        ++got;
    }
    for (int s = got; s < 3; ++s) g_watchIdx[s] = -1;
    if (!got) return false;
    for (int s = 0; s < 3; ++s) {
        g_writerCount[s] = 0;
        g_writerOverflow[s] = 0;
        for (auto& w : g_writers[s]) w = Writer{};
    }
    for (int s = 0; s < got; ++s) Dump(g_cand[g_watchIdx[s]], "before");
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
        if (!have) Log::get().note("camera hunt: hit %d (%p): no writes in the window.", g_watchIdx[s], reinterpret_cast<void*>(c.addr));
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
        Dump(c, "after");
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
            g_movedFrames = 0;
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
            if (g_movedFrames < 180) return;
            const LONG n = g_candCount < kMaxCandidates ? g_candCount : kMaxCandidates;
            int live = 0;
            for (LONG i = 0; i < n; ++i) {
                Candidate& c = g_cand[i];
                c.live = c.compared && c.matched * 10 >= c.compared * 8;
                if (c.live) ++live;
                Log::get().note("camera hunt: hit %ld %s the camera (%u of %u turned frames).", static_cast<long>(i),
                                c.live ? "FOLLOWS" : "does not follow", c.matched, c.compared);
            }
            if (!live) {
                Log::get().note("camera hunt: no hit follows the camera; done.");
                g_stage = Stage::Done;
                return;
            }
            g_watchNext = 0;
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
                Log::get().note("camera hunt: done; every hit that follows the camera was watched.");
            }
            return;
        default:
            return;
    }
}

}  // namespace edvr
