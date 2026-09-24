#include "stereo_mode_probe.h"

#include <windows.h>
#include <intrin.h>

#include <atomic>
#include <cstdint>
#include <cstring>

#include "../common/code_hook.h"
#include "../common/config.h"
#include "../common/log.h"
#include "journal_watch.h"

namespace edvr {
namespace {

// Build 332841 (transition_flash_prevent.cpp's identity pair).
constexpr uint32_t kExpectedTimestamp = 1788384820u;
constexpr uint32_t kExpectedImageSize = 104894464u;
constexpr uintptr_t kExecuteRva = 0x283AFB0u;
// push rbx; sub rsp,20h; mov rax,[rcx+8]; mov rbx,rcx
constexpr uint8_t kExecuteBytes[13] = {0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48,
                                       0x8B, 0x41, 0x08, 0x48, 0x8B, 0xD9};
constexpr int kMaxLines = 400;
// The activity's own lookup of the object it sets the mode on: a chain of
// virtual getters ending in a type-keyed lookup (0x869EC0), then the
// service's getter at vtable +0xA0. Repeated here, read-only, to name the
// setter (the target's vtable +0x148).
constexpr uintptr_t kLookupRva = 0x869EC0u;
constexpr uintptr_t kTypeIdRva = 0x5F02BB4u;
using LookupFn = uintptr_t(__fastcall*)(uintptr_t, uintptr_t, uint32_t);
using GetterFn = uintptr_t(__fastcall*)(uintptr_t);

using Execute = uintptr_t(__fastcall*)(uintptr_t, uintptr_t, uintptr_t, uintptr_t);

alignas(8) std::atomic<uintptr_t> g_gate{0};  // nonzero: the relay calls observed()
std::atomic<Execute> g_original{nullptr};
std::atomic<int> g_override{-1};
std::atomic<int> g_lines{0};
uintptr_t g_base = 0;
bool g_installTried = false;
CodeHook g_hook;

__declspec(noinline) bool sehReadParams(uintptr_t self, uintptr_t* params, uint32_t* mode) noexcept {
    __try {
        *params = *reinterpret_cast<const uintptr_t*>(self + 0x10);
        *mode = *params ? *reinterpret_cast<const uint32_t*>(*params + 8) : 0xFFFFFFFFu;
        return *params != 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

__declspec(noinline) bool sehResolveTarget(uintptr_t self, uintptr_t* tgt, uintptr_t* vtable,
                                             uintptr_t* setter) noexcept {
    __try {
        const uintptr_t owner = *reinterpret_cast<const uintptr_t*>(self + 8);
        if (!owner) return false;
        const uint32_t type = *reinterpret_cast<const uint32_t*>(g_base + kTypeIdRva);
        uintptr_t unused = 0;
        const uintptr_t svc =
            reinterpret_cast<LookupFn>(g_base + kLookupRva)(reinterpret_cast<uintptr_t>(&unused), owner + 0x40, type);
        if (!svc) return false;
        const uintptr_t svcVt = *reinterpret_cast<const uintptr_t*>(svc);
        const uintptr_t t = reinterpret_cast<GetterFn>(*reinterpret_cast<const uintptr_t*>(svcVt + 0xA0))(svc);
        if (!t) return false;
        *tgt = t;
        *vtable = *reinterpret_cast<const uintptr_t*>(t);
        *setter = *reinterpret_cast<const uintptr_t*>(*vtable + 0x148);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

__declspec(noinline) bool sehWriteMode(uintptr_t params, uint32_t mode) noexcept {
    __try {
        *reinterpret_cast<uint32_t*>(params + 8) = mode;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

__declspec(noinline) uintptr_t __fastcall observed(uintptr_t self, uintptr_t a2, uintptr_t a3, uintptr_t a4) {
    const uintptr_t caller = reinterpret_cast<uintptr_t>(_ReturnAddress());
    uintptr_t params = 0;
    uint32_t mode = 0;
    const bool read = sehReadParams(self, &params, &mode);
    const int override = g_override.load(std::memory_order_relaxed);
    const bool swap = read && override >= 0 && override <= 2 && static_cast<uint32_t>(override) != mode;
    if (g_lines.fetch_add(1, std::memory_order_relaxed) < kMaxLines) {
        const bool footKnown = journalOnFootKnown();
        uintptr_t tgt = 0, vtable = 0, setter = 0;
        const bool resolved = sehResolveTarget(self, &tgt, &vtable, &setter);
        auto rva = [](uintptr_t a) { return static_cast<unsigned long long>(a >= g_base ? a - g_base : a); };
        Log::get().note("stereo mode probe: SetIdentStereoRenderMode mode %d%s (caller +0x%llX, thread %lu, "
                        "journal: %s); target %p vtable +0x%llX setter +0x%llX%s%s",
                        read ? static_cast<int>(mode) : -1, read ? "" : " (parameters unreadable)",
                        rva(caller), GetCurrentThreadId(),
                        !footKnown ? "unknown" : (journalOnFoot() ? "on foot" : "not on foot"),
                        reinterpret_cast<void*>(tgt), rva(vtable), rva(setter),
                        resolved ? "" : " (target not resolved)", swap ? " -- OVERRIDDEN for this call" : "");
    }
    const Execute forward = g_original.load(std::memory_order_acquire);
    if (!swap) return forward(self, a2, a3, a4);
    sehWriteMode(params, static_cast<uint32_t>(override));
    const uintptr_t r = forward(self, a2, a3, a4);
    sehWriteMode(params, mode);
    return r;
}

// --- the relay (object_record_writer_hook.cpp's, verbatim in shape) ---------
constexpr size_t kRelayBytes = 44, kOriginalLiteral = 36;
uint8_t* g_relay = nullptr;

uint8_t* allocateRelay(uintptr_t target) noexcept {
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    const uintptr_t granularity = info.dwAllocationGranularity;
    const uintptr_t floor = reinterpret_cast<uintptr_t>(info.lpMinimumApplicationAddress);
    const uintptr_t ceiling = reinterpret_cast<uintptr_t>(info.lpMaximumApplicationAddress);
    const uintptr_t distance = uintptr_t(INT32_MAX) - 0x10000u;
    uintptr_t at = target > distance ? target - distance : floor;
    if (at < floor) at = floor;
    const uintptr_t limit = target > ceiling - distance ? ceiling : target + distance;
    while (at < limit) {
        MEMORY_BASIC_INFORMATION region{};
        if (!VirtualQuery(reinterpret_cast<void*>(at), &region, sizeof(region))) break;
        const uintptr_t start = reinterpret_cast<uintptr_t>(region.BaseAddress);
        if (region.RegionSize > UINTPTR_MAX - start) break;
        const uintptr_t end = start + region.RegionSize;
        if (region.State == MEM_FREE) {
            uintptr_t candidate = at > start ? at : start;
            if (candidate > UINTPTR_MAX - (granularity - 1)) break;
            candidate = (candidate + granularity - 1) & ~(granularity - 1);
            if (candidate < limit && candidate < end && end - candidate >= 4096) {
                auto* p = static_cast<uint8_t*>(
                    VirtualAlloc(reinterpret_cast<void*>(candidate), 4096, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE));
                if (p) return p;
            }
        }
        if (end <= at) break;
        at = end;
    }
    return nullptr;
}

void buildRelay(uint8_t* code, const void* gate, void* callback) noexcept {
    // mov rax,&gate; cmp qword ptr [rax],0; je original; jmp [callback];
    // original: jmp [trampoline].
    const uint8_t body[kRelayBytes] = {0x48, 0xB8, 0,    0,    0, 0, 0, 0, 0, 0, 0x48, 0x83, 0x38, 0, 0x74,
                                       0x0E, 0xFF, 0x25, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0,    0,    0, 0xFF,
                                       0x25, 0,    0,    0,    0, 0, 0, 0, 0, 0, 0, 0,    0};
    std::memcpy(code, body, sizeof(body));
    const uintptr_t gateAddress = reinterpret_cast<uintptr_t>(gate);
    const uintptr_t callbackAddress = reinterpret_cast<uintptr_t>(callback);
    std::memcpy(code + 2, &gateAddress, 8);
    std::memcpy(code + 22, &callbackAddress, 8);
}

bool prepareRelay(void* trampoline, void*) noexcept {
    const uintptr_t address = reinterpret_cast<uintptr_t>(trampoline);
    std::memcpy(g_relay + kOriginalLiteral, &address, 8);
    DWORD oldProtect = 0;
    if (!VirtualProtect(g_relay, 4096, PAGE_EXECUTE_READ, &oldProtect) ||
        !FlushInstructionCache(GetCurrentProcess(), g_relay, kRelayBytes))
        return false;
    g_original.store(reinterpret_cast<Execute>(trampoline), std::memory_order_release);
    return true;
}

__declspec(noinline) const char* checkTarget(uintptr_t base) noexcept {
    __try {
        uint32_t peOff = 0, timestamp = 0, imageSize = 0;
        std::memcpy(&peOff, reinterpret_cast<const void*>(base + 0x3C), 4);
        if (peOff > 0x1000) return "PE header offset implausible";
        std::memcpy(&timestamp, reinterpret_cast<const void*>(base + peOff + 8), 4);
        std::memcpy(&imageSize, reinterpret_cast<const void*>(base + peOff + 0x50), 4);
        if (timestamp != kExpectedTimestamp || imageSize != kExpectedImageSize)
            return "not build 332841 (PE timestamp/size mismatch)";
        if (std::memcmp(reinterpret_cast<const void*>(base + kExecuteRva), kExecuteBytes, sizeof(kExecuteBytes)) != 0)
            return "the function's first bytes are not the expected ones";
        return nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "a read faulted while checking the executable";
    }
}

void install() {
    g_installTried = true;
    g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    if (const char* why = checkTarget(g_base)) {
        Log::get().note("stereo mode probe: not installed: %s.", why);
        return;
    }
    const uintptr_t target = g_base + kExecuteRva;
    g_relay = allocateRelay(target);
    if (!g_relay) {
        Log::get().note("stereo mode probe: not installed: no memory for the relay near the game's code.");
        return;
    }
    buildRelay(g_relay, &g_gate, reinterpret_cast<void*>(&observed));
    if (!g_hook.install(reinterpret_cast<void*>(target), g_relay, nullptr, "stereo-mode-activity", &prepareRelay,
                        nullptr)) {
        g_original.store(nullptr, std::memory_order_release);
        VirtualFree(g_relay, 0, MEM_RELEASE);
        g_relay = nullptr;
        Log::get().note("stereo mode probe: not installed: the code hook refused (see above).");
        return;
    }
    // Process lifetime: the relay and trampoline are never freed; the gate
    // turns the observation off.
    Log::get().note("stereo mode probe: installed on SetIdentStereoRenderModeActivity's Execute (+0x%llX).",
                    static_cast<unsigned long long>(kExecuteRva));
}

}  // namespace

void stereoModeProbeConfigure(Config& cfg) {
    const bool on = cfg.getBool("experimental.stereo_mode_probe", false);
    const int override = cfg.getInt("experimental.stereo_mode_override", -1);
    if (on && !g_installTried) install();
    g_gate.store(on && g_original.load() ? 1 : 0, std::memory_order_release);
    if (override != g_override.load()) {
        Log::get().note(override >= 0 && override <= 2
                            ? "stereo mode probe: override: every SetIdentStereoRenderMode call gets mode %d."
                            : "stereo mode probe: override off (%d).",
                        override);
        g_override.store(override);
    }
}

}  // namespace edvr
