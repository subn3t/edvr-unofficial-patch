#include "mem_probe.h"

#include <windows.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "../common/config.h"
#include "../common/log.h"
#include "camera_hunt.h"
#include "hw_watch.h"

namespace edvr {

namespace {

uintptr_t g_base = 0;
uint64_t g_imageSize = 0;

std::string g_peekText, g_watchText;
int g_run = 0, g_repeatMs = 0, g_watchMs = 3000;
bool g_peekPending = false, g_watchPending = false;
ULONGLONG g_lastPeekMs = 0;

bool ReadOwn(uintptr_t at, void* out, SIZE_T bytes) {
    if (at < 0x10000) return false;
    SIZE_T got = 0;
    return ReadProcessMemory(GetCurrentProcess(), reinterpret_cast<const void*>(at), out, bytes, &got) &&
           got == bytes;
}

bool InImage(uint64_t v) { return v >= g_base && v < g_base + g_imageSize; }
bool UserPointer(uint64_t v) { return v >= 0x10000 && v < 0x800000000000ull && (v & 7) == 0; }

// <expr>: terms joined by + and -; a term is a hex number, base, [expr] (the
// qword there) or (expr).
struct Parser {
    const char* p;
    bool ok = true;

    void Skip() {
        while (*p == ' ' || *p == '\t') ++p;
    }
    uint64_t Expr() {
        uint64_t v = Term();
        for (;;) {
            Skip();
            if (*p == '+') {
                ++p;
                v += Term();
            } else if (*p == '-') {
                ++p;
                v -= Term();
            } else {
                return v;
            }
        }
    }
    uint64_t Term() {
        Skip();
        if (*p == '[' || *p == '(') {
            const char close = *p == '[' ? ']' : ')';
            const bool deref = *p == '[';
            ++p;
            const uint64_t a = Expr();
            Skip();
            if (*p != close) {
                ok = false;
                return 0;
            }
            ++p;
            if (!deref) return a;
            uint64_t v = 0;
            if (!ReadOwn(static_cast<uintptr_t>(a), &v, 8)) ok = false;
            return v;
        }
        if (strncmp(p, "base", 4) == 0) {
            p += 4;
            return g_base;
        }
        return Number();
    }
    uint64_t Number() {
        Skip();
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        const char* start = p;
        uint64_t v = 0;
        for (;; ++p) {
            const char c = *p;
            int d;
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
            else break;
            v = v * 16 + static_cast<uint64_t>(d);
        }
        if (p == start) ok = false;
        return v;
    }
};

// The items of a key: separated by ';'.
int Split(const std::string& text, std::string* out, int cap) {
    int n = 0;
    size_t at = 0;
    while (at <= text.size() && n < cap) {
        size_t end = text.find(';', at);
        if (end == std::string::npos) end = text.size();
        std::string item = text.substr(at, end - at);
        if (item.find_first_not_of(" \t") != std::string::npos) out[n++] = item;
        at = end + 1;
    }
    return n;
}

bool PlausibleDouble(uint64_t bits) {
    if (bits == 0) return true;
    const int exp = static_cast<int>((bits >> 52) & 0x7FF);
    return exp >= 1023 - 30 && exp <= 1023 + 30;
}

void Annotate(uint64_t q, char* out, size_t cap) {
    out[0] = 0;
    if (InImage(q)) {
        snprintf(out, cap, " [img+0x%llX]", static_cast<unsigned long long>(q - g_base));
        return;
    }
    uint64_t first = 0;
    if (UserPointer(q) && ReadOwn(static_cast<uintptr_t>(q), &first, 8) && InImage(first))
        snprintf(out, cap, " [obj vt img+0x%llX]", static_cast<unsigned long long>(first - g_base));
}

void Peek(const char* label, uintptr_t at, uint32_t len) {
    if (len > 0x2000) len = 0x2000;
    Log::get().note("probe: peek %s = %p, 0x%X bytes:", label, reinterpret_cast<void*>(at), len);
    for (uint32_t o = 0; o < len; o += 16) {
        uint64_t q[2];
        if (!ReadOwn(at + o, q, 16)) {
            Log::get().note("probe:   +%04X unreadable.", o);
            return;
        }
        float f[4];
        memcpy(f, q, 16);
        double d[2];
        memcpy(d, q, 16);
        char ds[2][32], an[2][48];
        for (int k = 0; k < 2; ++k) {
            if (PlausibleDouble(q[k])) snprintf(ds[k], sizeof(ds[k]), "%.9g", d[k]);
            else snprintf(ds[k], sizeof(ds[k]), "-");
            Annotate(q[k], an[k], sizeof(an[k]));
        }
        Log::get().note("probe:   +%04X  %016llX %016llX  f %.6g %.6g %.6g %.6g  d %s %s%s%s", o,
                        static_cast<unsigned long long>(q[0]), static_cast<unsigned long long>(q[1]), f[0], f[1], f[2],
                        f[3], ds[0], ds[1], an[0], an[1]);
    }
}

void RunPeeks() {
    std::string items[16];
    const int n = Split(g_peekText, items, 16);
    for (int i = 0; i < n; ++i) {
        Parser ps{items[i].c_str()};
        const uint64_t at = ps.Expr();
        ps.Skip();
        uint32_t len = 0x100;
        if (ps.ok && *ps.p) len = static_cast<uint32_t>(ps.Number());
        if (!ps.ok) {
            Log::get().note("probe: peek %d (%s): cannot evaluate.", i + 1, items[i].c_str());
            continue;
        }
        char label[160];
        snprintf(label, sizeof(label), "%d (%s)", i + 1, items[i].c_str());
        Peek(label, static_cast<uintptr_t>(at), len ? len : 0x100);
    }
}

// --- the find: every aligned qword equal to a value --------------------------
//
// An object is found by its vtable: probe_find = base+<vtable RVA> lists every
// instance of the class. The process's private read-write memory, stacks
// skipped, on a thread of its own; the hits logged when it is done.

constexpr int kMaxFindValues = 4, kMaxFindHits = 48;
uint64_t g_findValue[kMaxFindValues] = {};
std::string g_findItem[kMaxFindValues];
int g_findCount = 0;
uintptr_t g_findHits[kMaxFindValues][kMaxFindHits];
volatile LONG g_findHitCount[kMaxFindValues] = {};
volatile LONG g_findRunning = 0, g_findDone = 0;
std::string g_findText;
bool g_findPending = false;
ULONGLONG g_findStartMs = 0;

bool IsStackAllocation(uintptr_t allocationBase) {
    MEMORY_BASIC_INFORMATION m{};
    uintptr_t p = allocationBase;
    for (int i = 0; i < 64 && VirtualQuery(reinterpret_cast<const void*>(p), &m, sizeof(m)) == sizeof(m); ++i) {
        if (reinterpret_cast<uintptr_t>(m.AllocationBase) != allocationBase) break;
        if (m.State == MEM_COMMIT && (m.Protect & PAGE_GUARD)) return true;
        p = reinterpret_cast<uintptr_t>(m.BaseAddress) + m.RegionSize;
    }
    return false;
}

DWORD WINAPI FindProc(LPVOID) {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
    constexpr SIZE_T kChunk = 1u << 20;
    uint64_t* buf = static_cast<uint64_t*>(VirtualAlloc(nullptr, kChunk, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE));
    if (buf) {
        const uintptr_t bufLo = reinterpret_cast<uintptr_t>(buf), bufHi = bufLo + kChunk;
        MEMORY_BASIC_INFORMATION mbi{};
        uintptr_t p = 0x10000, lastAlloc = 0;
        bool lastStack = false;
        while (p < 0x7FFFFFFF0000ull &&
               VirtualQuery(reinterpret_cast<const void*>(p), &mbi, sizeof(mbi)) == sizeof(mbi)) {
            const uintptr_t base = reinterpret_cast<uintptr_t>(mbi.BaseAddress), end = base + mbi.RegionSize;
            const bool rw = (mbi.Protect & 0xFF) == PAGE_READWRITE && !(mbi.Protect & (PAGE_GUARD | PAGE_NOCACHE));
            if (mbi.State == MEM_COMMIT && mbi.Type == MEM_PRIVATE && rw && !(base < bufHi && end > bufLo)) {
                const uintptr_t alloc = reinterpret_cast<uintptr_t>(mbi.AllocationBase);
                if (alloc != lastAlloc) {
                    lastAlloc = alloc;
                    lastStack = IsStackAllocation(alloc);
                }
                for (uintptr_t at = base; !lastStack && at < end; at += kChunk) {
                    const SIZE_T n = static_cast<SIZE_T>(end - at < kChunk ? end - at : kChunk);
                    if (!ReadOwn(at, buf, n)) continue;
                    for (SIZE_T k = 0; k < n / 8; ++k)
                        for (int v = 0; v < g_findCount; ++v)
                            if (buf[k] == g_findValue[v]) {
                                const LONG i = InterlockedIncrement(&g_findHitCount[v]) - 1;
                                if (i < kMaxFindHits) g_findHits[v][i] = at + k * 8;
                            }
                }
            }
            if (end <= p) break;
            p = end;
        }
        VirtualFree(buf, 0, MEM_RELEASE);
    }
    InterlockedExchange(&g_findDone, 1);
    return 0;
}

void StartFind() {
    std::string items[kMaxFindValues];
    const int n = Split(g_findText, items, kMaxFindValues);
    g_findCount = 0;
    for (int i = 0; i < n; ++i) {
        Parser ps{items[i].c_str()};
        const uint64_t v = ps.Expr();
        if (!ps.ok) {
            Log::get().note("probe: find %d (%s): cannot evaluate.", i + 1, items[i].c_str());
            continue;
        }
        g_findItem[g_findCount] = items[i];
        g_findValue[g_findCount] = v;
        g_findHitCount[g_findCount] = 0;
        ++g_findCount;
    }
    if (!g_findCount) return;
    g_findDone = 0;
    HANDLE h = CreateThread(nullptr, 0, &FindProc, nullptr, 0, nullptr);
    if (!h) return;
    CloseHandle(h);
    g_findRunning = 1;
    g_findStartMs = GetTickCount64();
    Log::get().note("probe: finding %d value(s) in the game's memory...", g_findCount);
}

void EndFind() {
    g_findRunning = 0;
    for (int v = 0; v < g_findCount; ++v) {
        const LONG hits = g_findHitCount[v];
        Log::get().note("probe: find %d (%s = %016llX): %ld hit(s) in %llu ms.", v + 1, g_findItem[v].c_str(),
                        static_cast<unsigned long long>(g_findValue[v]), static_cast<long>(hits),
                        static_cast<unsigned long long>(GetTickCount64() - g_findStartMs));
        for (LONG i = 0; i < hits && i < kMaxFindHits; i += 6) {
            char line[512];
            int k = snprintf(line, sizeof(line), "probe:   at");
            for (LONG j = i; j < i + 6 && j < hits && j < kMaxFindHits; ++j)
                k += snprintf(line + k, sizeof(line) - k, " %p", reinterpret_cast<void*>(g_findHits[v][j]));
            Log::get().note("%s", line);
        }
    }
}

// --- the watch --------------------------------------------------------------

constexpr int kMaxHits = 8, kMaxFrames = 12;
struct Hit {
    volatile LONG count;
    uint32_t rip;
    GameFrame frames[kMaxFrames];  // the game's frames, each with its nonvolatile registers
    uint32_t n;
    uint64_t regs[16];  // rax rcx rdx rbx rsp rbp rsi rdi r8..r15 at the first
};
Hit g_hits[3][kMaxHits];
volatile LONG g_hitCount[3] = {}, g_overflow[3] = {};
uintptr_t g_watchAddr[3] = {};
uint8_t g_watchLen[3] = {};
bool g_watchReads[3] = {};
std::string g_watchItem[3];
PVOID g_veh = nullptr;
volatile LONG g_vehArmed = 0;
bool g_watching = false, g_swept = false;
ULONGLONG g_watchStartMs = 0;

LONG CALLBACK ProbeVeh(EXCEPTION_POINTERS* ep) {
    if (!ep || !ep->ExceptionRecord || !ep->ContextRecord || !g_vehArmed) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    CONTEXT& c = *ep->ContextRecord;
    const DWORD64 dr6 = c.Dr6;
    if (!(dr6 & 0xE)) return EXCEPTION_CONTINUE_SEARCH;
    c.Dr6 = dr6 & ~DWORD64(0xE);
    const uintptr_t rip = reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress);
    const uint32_t rva = InImage(rip) ? static_cast<uint32_t>(rip - g_base) : 0xFFFFFFFFu;
    for (int s = 0; s < 3; ++s) {
        if (!(dr6 & (DWORD64(2) << s))) continue;
        const LONG have = g_hitCount[s] < kMaxHits ? g_hitCount[s] : kMaxHits;
        bool found = false;
        for (LONG i = 0; i < have && !found; ++i)
            if (g_hits[s][i].rip == rva) {
                InterlockedIncrement(&g_hits[s][i].count);
                found = true;
            }
        if (found) continue;
        const LONG i = InterlockedIncrement(&g_hitCount[s]) - 1;
        if (i >= kMaxHits) {
            InterlockedIncrement(&g_overflow[s]);
            continue;
        }
        Hit& h = g_hits[s][i];
        h.n = unwindGameFrames(c, g_base, g_imageSize, h.frames, kMaxFrames);
        const DWORD64 regs[16] = {c.Rax, c.Rcx, c.Rdx, c.Rbx, c.Rsp, c.Rbp, c.Rsi, c.Rdi,
                                  c.R8,  c.R9,  c.R10, c.R11, c.R12, c.R13, c.R14, c.R15};
        for (int k = 0; k < 16; ++k) h.regs[k] = regs[k];
        h.count = 1;
        MemoryBarrier();
        h.rip = rva;
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

void StartWatch() {
    std::string items[3];
    const int n = Split(g_watchText, items, 3);
    uintptr_t addr[3] = {};
    uint8_t len[3] = {};
    bool reads[3] = {};
    int got = 0;
    for (int i = 0; i < n; ++i) {
        Parser ps{items[i].c_str()};
        const uint64_t at = ps.Expr();
        uint8_t l = 8;
        bool rw = false;
        ps.Skip();
        while (ps.ok && *ps.p) {
            if (strncmp(ps.p, "rw", 2) == 0) {
                rw = true;
                ps.p += 2;
            } else if (*ps.p == 'w') {
                ++ps.p;
            } else {
                l = static_cast<uint8_t>(ps.Number());
            }
            ps.Skip();
        }
        if (!ps.ok || (l != 1 && l != 2 && l != 4 && l != 8) || at % l != 0) {
            Log::get().note("probe: watch %d (%s): not a watchable address (len 1, 2, 4 or 8, aligned).", i + 1,
                            items[i].c_str());
            continue;
        }
        addr[got] = static_cast<uintptr_t>(at);
        len[got] = l;
        reads[got] = rw;
        g_watchItem[got] = items[i];
        ++got;
    }
    if (!got) return;
    for (int s = 0; s < 3; ++s) {
        g_watchAddr[s] = addr[s];
        g_watchLen[s] = len[s];
        g_watchReads[s] = reads[s];
        g_hitCount[s] = g_overflow[s] = 0;
        for (auto& h : g_hits[s]) h = Hit{};
    }
    for (int s = 0; s < got; ++s) {
        char label[160];
        snprintf(label, sizeof(label), "before watch %d (%s)", s + 1, g_watchItem[s].c_str());
        Peek(label, (addr[s] & ~uintptr_t(15)) - 0x20, 0x60);
    }
    if (!g_veh) g_veh = AddVectoredExceptionHandler(1, &ProbeVeh);
    InterlockedExchange(&g_vehArmed, 1);
    const int threads = hwWatchArm(addr, len, reads);
    g_watching = true;
    g_swept = false;
    g_watchStartMs = GetTickCount64();
    Log::get().note("probe: watching %d address(es) on %d threads for %d ms.", got, threads, g_watchMs);
}

void EndWatch() {
    hwWatchDisarm();
    InterlockedExchange(&g_vehArmed, 0);
    g_watching = false;
    for (int s = 0; s < 3; ++s) {
        if (!g_watchAddr[s]) continue;
        const char* what = g_watchReads[s] ? "touched" : "written";
        const LONG have = g_hitCount[s] < kMaxHits ? g_hitCount[s] : kMaxHits;
        if (!have)
            Log::get().note("probe: watch %d (%s = %p): not %s in the window.", s + 1, g_watchItem[s].c_str(),
                            reinterpret_cast<void*>(g_watchAddr[s]), what);
        for (LONG i = 0; i < have; ++i) {
            const Hit& h = g_hits[s][i];
            char frames[300];
            int k = 0;
            frames[0] = 0;
            for (uint32_t f = 0; f < h.n && k < static_cast<int>(sizeof(frames)) - 16; ++f)
                k += snprintf(frames + k, sizeof(frames) - k, " +0x%X", h.frames[f].rva);
            Log::get().note("probe: watch %d (%s = %p) %s %ld times at +0x%X (the instruction before); stack:%s",
                            s + 1, g_watchItem[s].c_str(), reinterpret_cast<void*>(g_watchAddr[s]), what,
                            static_cast<long>(h.count), h.rip, frames);
            const uint64_t* r = h.regs;
            Log::get().note("probe:   first time: rax %llX rcx %llX rdx %llX rbx %llX rsp %llX rbp %llX rsi %llX rdi "
                            "%llX r8 %llX r9 %llX r10 %llX r11 %llX r12 %llX r13 %llX r14 %llX r15 %llX",
                            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13],
                            r[14], r[15]);
            // Each frame's own registers: a caller's object where it held it.
            for (uint32_t f = 0; f < h.n && f < 8; ++f) {
                const GameFrame& g = h.frames[f];
                char an[8][48];
                const uint64_t v[8] = {g.rbx, g.rbp, g.rsi, g.rdi, g.r12, g.r13, g.r14, g.r15};
                for (int i = 0; i < 8; ++i) Annotate(v[i], an[i], sizeof(an[i]));
                Log::get().note("probe:   frame +0x%X: rbx %llX%s rbp %llX%s rsi %llX%s rdi %llX%s r12 %llX%s r13 %llX%s "
                                "r14 %llX%s r15 %llX%s",
                                g.rva, v[0], an[0], v[1], an[1], v[2], an[2], v[3], an[3], v[4], an[4], v[5], an[5], v[6],
                                an[6], v[7], an[7]);
            }
        }
        if (g_overflow[s])
            Log::get().note("probe: watch %d: %ld hits by more code than kept.", s + 1,
                            static_cast<long>(g_overflow[s]));
        char label[160];
        snprintf(label, sizeof(label), "after watch %d (%s)", s + 1, g_watchItem[s].c_str());
        Peek(label, (g_watchAddr[s] & ~uintptr_t(15)) - 0x20, 0x60);
    }
    Log::get().note("probe: watch done.");
}

}  // namespace

void memProbeConfigure(Config& cfg) {
    if (!g_base) {
        g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
        const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
        const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew);
        g_imageSize = nt->OptionalHeader.SizeOfImage;
    }
    const std::string peek = cfg.getString("experimental.probe_peek", "");
    const std::string watch = cfg.getString("experimental.probe_watch", "");
    const std::string find = cfg.getString("experimental.probe_find", "");
    const int run = cfg.getInt("experimental.probe_run", 0);
    const int repeat = cfg.getInt("experimental.probe_repeat_ms", 0);
    const int watchMs = cfg.getInt("experimental.probe_watch_ms", 3000);
    const bool again = run != g_run;
    if (peek != g_peekText || (again && !peek.empty())) g_peekPending = !peek.empty();
    if (watch != g_watchText || (again && !watch.empty())) g_watchPending = !watch.empty();
    if (find != g_findText || (again && !find.empty())) g_findPending = !find.empty();
    g_peekText = peek;
    g_watchText = watch;
    g_findText = find;
    g_run = run;
    g_repeatMs = repeat <= 0 ? 0 : (repeat < 100 ? 100 : repeat);
    g_watchMs = watchMs < 250 ? 250 : (watchMs > 30000 ? 30000 : watchMs);
    if (g_peekPending || g_watchPending || g_findPending)
        Log::get().note("probe: %s%s%s at the next frame (image base %p).", g_peekPending ? "peek " : "",
                        g_watchPending ? "watch " : "", g_findPending ? "find " : "",
                        reinterpret_cast<void*>(g_base));
}

void memProbeFrame() {
    if (g_findRunning && g_findDone) EndFind();
    if (g_findPending && !g_findRunning) {
        g_findPending = false;
        StartFind();
    }
    if (!g_peekPending && !g_watchPending && !g_watching && !g_repeatMs) return;
    const ULONGLONG now = GetTickCount64();
    if (g_peekPending || (g_repeatMs && !g_peekText.empty() && now - g_lastPeekMs >= static_cast<ULONGLONG>(g_repeatMs))) {
        g_peekPending = false;
        g_lastPeekMs = now;
        RunPeeks();
    }
    if (g_watching) {
        if (!g_swept && now - g_watchStartMs > 1000) {
            hwWatchSweep();
            g_swept = true;
        }
        if (now - g_watchStartMs >= static_cast<ULONGLONG>(g_watchMs)) EndWatch();
    } else if (g_watchPending && !cameraHuntWatching()) {
        g_watchPending = false;
        StartWatch();
    }
}

}  // namespace edvr
