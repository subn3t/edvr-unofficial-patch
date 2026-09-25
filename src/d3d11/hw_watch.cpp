#include "hw_watch.h"

#include <tlhelp32.h>

#include <new>

namespace edvr {

namespace {

uintptr_t g_addr[3] = {};
uint8_t g_len[3] = {};
bool g_reads[3] = {};
DWORD g_armedTids[512];
int g_armedTidCount = 0;

// Dr7 LEN encoding: 00 one byte, 01 two, 11 four, 10 eight.
DWORD64 LenBits(uint8_t len) { return len == 8 ? 2 : (len == 4 ? 3 : (len == 2 ? 1 : 0)); }

// Dr7 for slots 1..3: local enable (bit 2i), RW at 16+4i (01 writes, 11
// reads and writes), LEN at 18+4i. Other bits pass through.
DWORD64 ComposeDr7(DWORD64 dr7, bool arm) {
    for (int i = 1; i <= 3; ++i) {
        dr7 &= ~(DWORD64(1) << (2 * i));
        dr7 &= ~(DWORD64(0xF) << (16 + 4 * i));
        if (arm && g_addr[i - 1])
            dr7 |= (DWORD64(1) << (2 * i)) | (DWORD64(g_reads[i - 1] ? 3 : 1) << (16 + 4 * i)) |
                   (LenBits(g_len[i - 1]) << (18 + 4 * i));
    }
    return dr7;
}

bool SetThreadWatch(DWORD tid, bool arm) {
    HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_SET_CONTEXT, FALSE, tid);
    if (!h) return false;
    bool ok = false;
    if (SuspendThread(h) != static_cast<DWORD>(-1)) {
        CONTEXT ctx{};
        ctx.ContextFlags = CONTEXT_DEBUG_REGISTERS;
        if (GetThreadContext(h, &ctx)) {
            ctx.Dr7 = ComposeDr7(ctx.Dr7, arm);
            if (arm) {
                ctx.Dr1 = g_addr[0];
                ctx.Dr2 = g_addr[1];
                ctx.Dr3 = g_addr[2];
            }
            ok = SetThreadContext(h, &ctx) != FALSE;
        }
        ResumeThread(h);
    }
    CloseHandle(h);
    return ok;
}

struct SelfArgs {
    DWORD tid;
    bool arm;
};
DWORD WINAPI SelfWatchProc(LPVOID p) {
    SelfArgs* a = static_cast<SelfArgs*>(p);
    SetThreadWatch(a->tid, a->arm);
    delete a;
    return 0;
}

// A thread cannot set its own context while running: the calling thread is
// set from a helper thread.
bool SetWatchOn(DWORD tid, bool arm) {
    if (tid != GetCurrentThreadId()) return SetThreadWatch(tid, arm);
    auto* a = new (std::nothrow) SelfArgs{tid, arm};
    if (!a) return false;
    HANDLE h = CreateThread(nullptr, 0, &SelfWatchProc, a, 0, nullptr);
    if (!h) {
        delete a;
        return false;
    }
    WaitForSingleObject(h, 2000);
    CloseHandle(h);
    return true;
}

}  // namespace

void hwWatchSweep() {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te{};
    te.dwSize = sizeof(te);
    const DWORD pid = GetCurrentProcessId();
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != pid) continue;
            bool have = false;
            for (int i = 0; i < g_armedTidCount; ++i)
                if (g_armedTids[i] == te.th32ThreadID) have = true;
            if (have || g_armedTidCount >= 512) continue;
            if (SetWatchOn(te.th32ThreadID, true)) g_armedTids[g_armedTidCount++] = te.th32ThreadID;
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
}

int hwWatchArm(const uintptr_t addr[3], const uint8_t len[3], const bool reads[3]) {
    hwWatchDisarm();
    for (int i = 0; i < 3; ++i) {
        g_addr[i] = addr[i];
        g_len[i] = len[i];
        g_reads[i] = reads && reads[i];
    }
    hwWatchSweep();
    return g_armedTidCount;
}

void hwWatchDisarm() {
    for (int i = 0; i < g_armedTidCount; ++i) SetWatchOn(g_armedTids[i], false);
    g_armedTidCount = 0;
}

__declspec(noinline) uint32_t unwindGameStack(const CONTEXT& start, uintptr_t base, uint64_t size, uint32_t* out,
                                              uint32_t cap) noexcept {
    CONTEXT c = start;
    uint32_t found = 0;
    for (uint32_t step = 0; step < 40 && found < cap; ++step) {
        if (!c.Rip) break;
        DWORD64 imageBase = 0;
        PRUNTIME_FUNCTION function = nullptr;
        __try {
            function = RtlLookupFunctionEntry(c.Rip, &imageBase, nullptr);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            break;
        }
        const DWORD64 oldRip = c.Rip, oldRsp = c.Rsp;
        if (function) {
            DWORD64 establisher = 0;
            PVOID handlerData = nullptr;
            __try {
                RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, c.Rip, function, &c, &handlerData, &establisher,
                                 nullptr);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                break;
            }
        } else {
            uintptr_t next = 0;
            __try {
                next = *reinterpret_cast<const uintptr_t*>(c.Rsp);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                break;
            }
            c.Rip = next;
            c.Rsp += sizeof(uintptr_t);
        }
        if (c.Rsp <= oldRsp || c.Rip == oldRip) break;
        if (c.Rip >= base && c.Rip < base + size) out[found++] = static_cast<uint32_t>(c.Rip - base);
    }
    return found;
}

__declspec(noinline) uint32_t unwindGameFrames(const CONTEXT& start, uintptr_t base, uint64_t size, GameFrame* out,
                                               uint32_t cap) noexcept {
    CONTEXT c = start;
    uint32_t found = 0;
    const auto note = [&](const CONTEXT& k) {
        if (found >= cap || k.Rip < base || k.Rip >= base + size) return;
        out[found++] = GameFrame{static_cast<uint32_t>(k.Rip - base), k.Rbx, k.Rbp, k.Rsi, k.Rdi,
                                 k.R12, k.R13, k.R14, k.R15};
    };
    note(c);
    for (uint32_t step = 0; step < 40 && found < cap; ++step) {
        if (!c.Rip) break;
        DWORD64 imageBase = 0;
        PRUNTIME_FUNCTION function = nullptr;
        __try {
            function = RtlLookupFunctionEntry(c.Rip, &imageBase, nullptr);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            break;
        }
        const DWORD64 oldRip = c.Rip, oldRsp = c.Rsp;
        if (function) {
            DWORD64 establisher = 0;
            PVOID handlerData = nullptr;
            __try {
                RtlVirtualUnwind(UNW_FLAG_NHANDLER, imageBase, c.Rip, function, &c, &handlerData, &establisher,
                                 nullptr);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                break;
            }
        } else {
            uintptr_t next = 0;
            __try {
                next = *reinterpret_cast<const uintptr_t*>(c.Rsp);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                break;
            }
            c.Rip = next;
            c.Rsp += sizeof(uintptr_t);
        }
        if (c.Rsp <= oldRsp || c.Rip == oldRip) break;
        note(c);
    }
    return found;
}

}  // namespace edvr
