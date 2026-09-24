#include "stereo_mode_probe.h"

#include <windows.h>
#include <intrin.h>
#include <tlhelp32.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../common/code_hook.h"
#include "../common/config.h"
#include "../common/frame_flag.h"
#include "../common/log.h"
#include "hw_watch.h"
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
std::atomic<uintptr_t> g_tgt{0};  // the stereo mode's target, learned at the activity's call
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
        if (resolved) g_tgt.store(tgt, std::memory_order_release);
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

// --- the ship/foot differ -----------------------------------------------------
//
// The setter (+0x2875110) writes the mode at +0x118 of two views reached from
// the target: P = tgt->[0x248]->[0x1180], R = P->[0x2CB8], view = get(R->[0x90])
// and get(R->[0x98]), get(x) = x->[0x2D0]->[0x30]. A few seconds after each
// change of the journal's on-foot state, those objects are snapshotted; the
// small integers, flags and pointers that differ from the other state's
// snapshot are logged. Whatever flips when you step out of the ship is the
// switch, or the way to it.
struct Region {
    const char* name;
    uint32_t bytes;
};
constexpr Region kRegions[] = {{"tgt", 0x400}, {"P", 0x4000}, {"R", 0x800}, {"viewA", 0x400}, {"viewB", 0x400}};
constexpr int kRegionCount = sizeof(kRegions) / sizeof(kRegions[0]);
struct Snap {
    bool valid = false;
    uintptr_t addr[kRegionCount] = {};
    bool read[kRegionCount] = {};
    uint8_t bytes[kRegionCount][0x4000];
};
Snap g_snap[2];  // [0] not on foot, [1] on foot
bool g_footSeen = false, g_lastFoot = false, g_snapPending = false;
ULONGLONG g_changeMs = 0;

__declspec(noinline) uintptr_t sehRead64(uintptr_t a) noexcept {
    __try {
        return a ? *reinterpret_cast<const uintptr_t*>(a) : 0;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
}

__declspec(noinline) bool sehCopy(void* dst, uintptr_t src, uint32_t n) noexcept {
    __try {
        std::memcpy(dst, reinterpret_cast<const void*>(src), n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

uintptr_t ViewOf(uintptr_t x) {
    const uintptr_t h = sehRead64(x ? x + 0x2D0 : 0);
    return sehRead64(h ? h + 0x30 : 0);
}

void TakeSnap(Snap& snap, uintptr_t tgt) {
    const uintptr_t p = sehRead64(sehRead64(tgt + 0x248) + 0x1180);
    const uintptr_t r = sehRead64(p ? p + 0x2CB8 : 0);
    const uintptr_t a[kRegionCount] = {tgt, p, r, ViewOf(sehRead64(r ? r + 0x90 : 0)),
                                       ViewOf(sehRead64(r ? r + 0x98 : 0))};
    for (int i = 0; i < kRegionCount; ++i) {
        snap.addr[i] = a[i];
        snap.read[i] = a[i] && sehCopy(snap.bytes[i], a[i], kRegions[i].bytes);
    }
    snap.valid = true;
}

// The render pipelines (the builder at +0x28A2AE4): R->[0x90] and R->[0x98],
// type at +8 (0 Primary, 1 Secondary, 2 Thumbnail, 3 Auxiliary), size at
// +0x320/+0x324, features at +0x270..+0x2F0, each with its enabled byte at
// +0x20 and its owner at +0x18.
constexpr const char* kFeatureNames[] = {
    "DeferredShading?", "TiledMarch?", "?", "ReducedSize", "Forward", "Blur", "Restore", "DepthOfField", "Bloom",
    "HDR", "AntiAliasing", "Scaling", "UI", "PostGUIDepthPass", "PostGUI", "Cinema", "DebugOverlay"};

__declspec(noinline) uint8_t sehRead8(uintptr_t a) noexcept {
    __try {
        return a ? *reinterpret_cast<const uint8_t*>(a) : 0xFF;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0xFE;
    }
}

__declspec(noinline) uint32_t sehRead32(uintptr_t a) noexcept {
    __try {
        return a ? *reinterpret_cast<const uint32_t*>(a) : 0xFFFFFFFFu;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return 0xFFFFFFFEu;
    }
}

void LogPipelines(uintptr_t tgt, bool foot) {
    const uintptr_t p = sehRead64(sehRead64(tgt + 0x248) + 0x1180);
    const uintptr_t r = sehRead64(p ? p + 0x2CB8 : 0);
    for (int k = 0; k < 2; ++k) {
        const uintptr_t pl = sehRead64(r ? r + 0x90 + 8 * k : 0);
        if (!pl) continue;
        char line[1024];
        int n = snprintf(line, sizeof(line), "stereo mode probe: %s pipeline R+0x%X %p type %u size %ux%u; enabled:",
                         foot ? "on foot" : "in ship", 0x90 + 8 * k, reinterpret_cast<void*>(pl), sehRead32(pl + 8),
                         sehRead32(pl + 0x320), sehRead32(pl + 0x324));
        for (int f = 0; f < 17 && n > 0 && n < static_cast<int>(sizeof(line)) - 40; ++f) {
            const uintptr_t feat = sehRead64(pl + 0x270 + 8 * f);
            if (!feat) continue;
            n += snprintf(line + n, sizeof(line) - n, " %s=%u", kFeatureNames[f], sehRead8(feat + 0x20));
        }
        Log::get().note("%s", line);
    }
}


// --- the write-watch: who flips the features at a ship/foot change ----------
//
// The pipeline log showed the switch: on foot the Secondary pipeline's scene
// features are all disabled and both pipelines' Cinema is enabled; in the
// ship the reverse. Hardware data breakpoints (DR1-DR3: write, one byte) on
// three of those enabled bytes catch the code that writes them, with the
// game's own frames unwound -- the decision is in that stack.
constexpr int kWatchSlots = 3;  // DR1..DR3 (DR0 is pose_reader_watch's)
const char* const kWatchNames[kWatchSlots] = {"Secondary Cinema", "Secondary scene feature 0", "Primary Cinema"};
uintptr_t g_watchAddr[kWatchSlots] = {};
bool g_watchArmed = false, g_watchDone = false, g_watchWanted = false;
ULONGLONG g_watchArmedMs = 0, g_watchSweepMs = 0;
PVOID g_watchVeh = nullptr;
uint64_t g_gameSize = 0;

constexpr int kMaxFrames = 14, kMaxHits = 48;
struct Hit {
    int slot;
    uint8_t value;
    DWORD tid;
    uint32_t rip;
    uint32_t frames[kMaxFrames];
    uint32_t n;
};
Hit g_hits[kMaxHits];
volatile LONG g_hitCount = 0, g_hitsLogged = 0, g_hitsDropped = 0, g_hitsSame = 0;
// The value each watched byte had at its last write: a write of the same
// value (a per-frame re-apply) is counted, not recorded.
volatile uint8_t g_lastValue[kWatchSlots] = {0xFF, 0xFF, 0xFF};

uint32_t unwindGame(const CONTEXT& start, uint32_t* out, uint32_t cap) noexcept {
    return unwindGameStack(start, g_base, g_gameSize, out, cap);
}

LONG CALLBACK WatchVeh(EXCEPTION_POINTERS* ep) {
    if (!ep || !ep->ExceptionRecord || !ep->ContextRecord) return EXCEPTION_CONTINUE_SEARCH;
    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP) return EXCEPTION_CONTINUE_SEARCH;
    const DWORD64 dr6 = ep->ContextRecord->Dr6;
    if (!(dr6 & 0xE)) return EXCEPTION_CONTINUE_SEARCH;  // B1..B3: ours
    ep->ContextRecord->Dr6 = dr6 & ~DWORD64(0xE);
    for (int i = 0; i < kWatchSlots; ++i) {
        if (!(dr6 & (DWORD64(2) << i))) continue;
        const uint8_t value = *reinterpret_cast<volatile const uint8_t*>(g_watchAddr[i]);
        if (value == g_lastValue[i]) {
            InterlockedIncrement(&g_hitsSame);
            continue;
        }
        g_lastValue[i] = value;
        const LONG idx = InterlockedIncrement(&g_hitCount) - 1;
        if (idx >= kMaxHits) {
            InterlockedIncrement(&g_hitsDropped);
            continue;
        }
        Hit& h = g_hits[idx];
        h.slot = i;
        h.value = *reinterpret_cast<volatile const uint8_t*>(g_watchAddr[i]);
        h.tid = GetCurrentThreadId();
        const uintptr_t rip = reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress);
        h.rip = (rip >= g_base && rip < g_base + g_gameSize) ? static_cast<uint32_t>(rip - g_base) : 0xFFFFFFFFu;
        h.n = unwindGame(*ep->ContextRecord, h.frames, kMaxFrames);
    }
    return EXCEPTION_CONTINUE_EXECUTION;
}

void DisarmWatch() {
    hwWatchDisarm();
    g_watchArmed = false;
    g_watchDone = true;
}

void ArmWatch(uintptr_t tgt) {
    const uintptr_t p = sehRead64(sehRead64(tgt + 0x248) + 0x1180);
    const uintptr_t r = sehRead64(p ? p + 0x2CB8 : 0);
    const uintptr_t primary = sehRead64(r ? r + 0x90 : 0), secondary = sehRead64(r ? r + 0x98 : 0);
    const uintptr_t secCinema = sehRead64(secondary ? secondary + 0x2E8 : 0);
    const uintptr_t secScene0 = sehRead64(secondary ? secondary + 0x270 : 0);
    const uintptr_t priCinema = sehRead64(primary ? primary + 0x2E8 : 0);
    if (!secCinema || !secScene0 || !priCinema) return;
    g_watchAddr[0] = secCinema + 0x20;
    g_watchAddr[1] = secScene0 + 0x20;
    g_watchAddr[2] = priCinema + 0x20;
    for (int i = 0; i < kWatchSlots; ++i) g_lastValue[i] = *reinterpret_cast<volatile const uint8_t*>(g_watchAddr[i]);
    {
        const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(g_base);
        const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(g_base + dos->e_lfanew);
        g_gameSize = nt->OptionalHeader.SizeOfImage;
    }
    g_watchVeh = AddVectoredExceptionHandler(1, &WatchVeh);
    if (!g_watchVeh) {
        g_watchDone = true;
        return;
    }
    const uint8_t lens[kWatchSlots] = {1, 1, 1};
    const int armed = hwWatchArm(g_watchAddr, lens);
    g_watchArmed = true;
    g_watchArmedMs = g_watchSweepMs = GetTickCount64();
    Log::get().note("stereo mode probe: write-watch armed on %d threads: Secondary Cinema %p, Secondary scene "
                    "feature 0 %p, Primary Cinema %p (enabled bytes).",
                    armed, reinterpret_cast<void*>(g_watchAddr[0]), reinterpret_cast<void*>(g_watchAddr[1]),
                    reinterpret_cast<void*>(g_watchAddr[2]));
}

void WatchFrame(uintptr_t tgt) {
    if (g_watchDone) return;
    if (!g_watchArmed) {
        ArmWatch(tgt);
        return;
    }
    const ULONGLONG now = GetTickCount64();
    const LONG count = g_hitCount < kMaxHits ? g_hitCount : kMaxHits;
    while (g_hitsLogged < count) {
        const Hit& h = g_hits[g_hitsLogged];
        char frames[400];
        int n = 0;
        frames[0] = 0;
        for (uint32_t i = 0; i < h.n && n < static_cast<int>(sizeof(frames)) - 16; ++i)
            n += snprintf(frames + n, sizeof(frames) - n, " +0x%X", h.frames[i]);
        Log::get().note("stereo mode probe: WRITE %s -> %u at +0x%X (thread %lu, journal %s); stack:%s",
                        kWatchNames[h.slot], h.value, h.rip, h.tid,
                        !journalOnFootKnown() ? "unknown" : (journalOnFoot() ? "on foot" : "not on foot"), frames);
        ++g_hitsLogged;
    }
    if (now - g_watchSweepMs > 2000) {
        g_watchSweepMs = now;
        hwWatchSweep();
    }
    if (now - g_watchArmedMs > 900000 || g_hitCount >= kMaxHits) {
        DisarmWatch();
        Log::get().note("stereo mode probe: write-watch disarmed (%ld changes seen, %ld not recorded, %ld "
                        "same-value writes).",
                        static_cast<long>(g_hitCount), static_cast<long>(g_hitsDropped),
                        static_cast<long>(g_hitsSame));
    }
}

void StereoFrame();  // the on-foot stereo patch, below

bool LooksLikePointer(uint64_t v) { return v >= 0x10000000000ull && v < 0x800000000000ull; }

void LogDiff(const Snap& from, const Snap& to, bool toFoot) {
    const char* dir = toFoot ? "ship -> foot" : "foot -> ship";
    int lines = 0;
    for (int i = 0; i < kRegionCount; ++i) {
        const Region& reg = kRegions[i];
        if (from.addr[i] != to.addr[i])
            Log::get().note("stereo mode probe: diff %s: %s moved %p -> %p.", dir, reg.name,
                            reinterpret_cast<void*>(from.addr[i]), reinterpret_cast<void*>(to.addr[i]));
        if (!from.read[i] || !to.read[i]) {
            Log::get().note("stereo mode probe: diff %s: %s unreadable (%d/%d).", dir, reg.name, from.read[i],
                            to.read[i]);
            continue;
        }
        for (uint32_t k = 0; k + 8 <= reg.bytes && lines < 240; k += 4) {
            uint32_t a, b;
            std::memcpy(&a, from.bytes[i] + k, 4);
            std::memcpy(&b, to.bytes[i] + k, 4);
            if (a == b) continue;
            if (k % 8 == 0) {
                uint64_t qa, qb;
                std::memcpy(&qa, from.bytes[i] + k, 8);
                std::memcpy(&qb, to.bytes[i] + k, 8);
                if (LooksLikePointer(qa) || LooksLikePointer(qb)) {
                    Log::get().note("stereo mode probe: diff %s: %s+0x%X pointer %p -> %p", dir, reg.name, k,
                                    reinterpret_cast<void*>(qa), reinterpret_cast<void*>(qb));
                    ++lines;
                    k += 4;
                    continue;
                }
            }
            if (a < 0x10000 && b < 0x10000) {
                Log::get().note("stereo mode probe: diff %s: %s+0x%X %u -> %u (bytes %08X -> %08X)", dir, reg.name,
                                k, a, b, a, b);
                ++lines;
            }
        }
    }
    uint32_t va[3] = {}, vb[3] = {};
    if (to.read[3]) std::memcpy(va, to.bytes[3] + 0x110, 12);
    if (to.read[4]) std::memcpy(vb, to.bytes[4] + 0x110, 12);
    Log::get().note("stereo mode probe: now %s: viewA +0x110 %u, +0x118 (mode) %u, +0x11C %08X; viewB %u, %u, "
                    "%08X (%d lines of difference).",
                    toFoot ? "on foot" : "not on foot", va[0], va[1], va[2], vb[0], vb[1], vb[2], lines);
}

}  // namespace

void stereoModeProbeFrame() {
    StereoFrame();
    const uintptr_t tgt = g_tgt.load(std::memory_order_acquire);
    if (!tgt || !g_gate.load(std::memory_order_relaxed)) return;
    if (g_watchWanted || g_watchArmed) WatchFrame(tgt);
    if (!journalOnFootKnown()) return;
    const bool foot = journalOnFoot();
    const ULONGLONG now = GetTickCount64();
    if (!g_footSeen || foot != g_lastFoot) {
        g_footSeen = true;
        g_lastFoot = foot;
        g_changeMs = now;
        g_snapPending = true;
    }
    if (!g_snapPending || now - g_changeMs < 5000) return;
    g_snapPending = false;
    Snap& snap = g_snap[foot ? 1 : 0];
    TakeSnap(snap, tgt);
    LogPipelines(tgt, foot);
    const Snap& other = g_snap[foot ? 0 : 1];
    if (other.valid) LogDiff(other, snap, foot);
    else
        Log::get().note("stereo mode probe: first snapshot taken (%s); the diff comes at the next change.",
                        foot ? "on foot" : "not on foot");
}

namespace {

// --- on-foot stereo: keep the display mode at HMD stereo ----------------------
//
// The write-watch (flight 005341) caught the switch: at a ship/foot change the
// display-mode updater (+0x2890B30) finds P->[0x3698] (the wanted mode) unlike
// P->[0x3694] (the current one), applies it, and the feature set follows --
// Secondary's scene features off and Cinema on both eyes for mode 5 (on foot),
// the reverse for mode 3 (the ship, HMD stereo, the Settings.xml
// StereoscopicMode). The wanted mode is copied every frame at +0x281B052:
//
//     +0x281B04F  8B 40 20            mov eax,[rax+20h]
//     +0x281B052  89 81 98 36 00 00   mov [rcx+3698h],eax
//
// experimental.onfoot_stereo replaces that store with a jump to a stub that
// stores 3 where the game would store 5, so the updater never starts the
// switch and the engine keeps rendering both eyes. A flag in our own memory
// turns it on and off live; the patch itself stays for the session.
constexpr uintptr_t kStoreRva = 0x281B052u;
constexpr uint8_t kStoreBytes[9] = {0x8B, 0x40, 0x20, 0x89, 0x81, 0x98, 0x36, 0x00, 0x00};  // from +0x281B04F
uint8_t* g_stereoStub = nullptr;
volatile uint8_t* g_stereoData = nullptr;  // +0 flag, +4 substitutions, +8 the function's rsi
bool g_stereoTried = false, g_stereoPatched = false;
uint32_t g_stereoLastCount = 0;
// Frame boundaries since the stub last put a 3 where the game asked for 5.
// The game asks for 5 every frame it is on foot, so a count that moves is
// the ENGINE saying on foot -- right from a load on foot, when the journal
// still holds the last session's Embark (flight of 2026-09-24 15:36).
uint32_t g_quietFrames = ~0u;
bool g_swapWanted = false;

// THE LOAD-IN KICK (experimental.onfoot_stereo_kick). Loaded straight onto a
// planet, the game renders on foot but does not ask for its on-foot display
// mode until the camera's state changes -- a sprint, 47 s after LoadGame in
// the flight of 2026-09-24 17:18 -- and until it asks, the eye pipelines
// render at the headset's size and the stereo stays off. The request is
// X->[0x20], X = [rsi+0x7F8] in the function the stub sits in (+0x281B04F
// reads it). With Status.json saying on foot for three seconds and no request
// yet, 5 is written there once, as the game itself would; only within 15 s of
// the on-foot flag rising, so a later external camera (which the flag also
// covers) is never touched. Each change of the request is logged.
bool g_kickWanted = true;
bool g_footWas = false, g_kickedThisStretch = false, g_substitutedThisStretch = false;
ULONGLONG g_footSinceMs = 0;
uint32_t g_modeWas = ~0u;
int g_modeLogs = 0;

__declspec(noinline) bool sehWrite32(uintptr_t a, uint32_t v) noexcept {
    __try {
        if (!a) return false;
        *reinterpret_cast<volatile uint32_t*>(a) = v;
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

uintptr_t RequestAddr() {
    const uintptr_t self = *reinterpret_cast<volatile const uintptr_t*>(g_stereoData + 8);
    const uintptr_t x = sehRead64(self ? self + 0x7F8 : 0);
    return x ? x + 0x20 : 0;
}

void KickFrame(bool substituted) {
    const uintptr_t at = RequestAddr();
    uint32_t mode = ~0u;
    if (at && !sehCopy(&mode, at, 4)) mode = ~0u;
    const bool foot = journalOnFootKnown() && journalOnFoot();
    if (mode != g_modeWas && g_modeLogs < 40) {
        ++g_modeLogs;
        Log::get().note("onfoot stereo: the game's requested display mode %d -> %d (Status.json: %s).",
                        static_cast<int>(g_modeWas), static_cast<int>(mode),
                        !journalOnFootKnown() ? "unknown" : (foot ? "on foot" : "not on foot"));
    }
    g_modeWas = mode;
    const ULONGLONG now = GetTickCount64();
    if (foot && !g_footWas) {
        g_footSinceMs = now;
        g_kickedThisStretch = g_substitutedThisStretch = false;
    }
    g_footWas = foot;
    if (substituted) g_substitutedThisStretch = true;
    if (!g_kickWanted || !*g_stereoData || !foot || g_kickedThisStretch || g_substitutedThisStretch || mode != 3)
        return;
    const ULONGLONG since = now - g_footSinceMs;
    if (since < 3000 || since > 15000) return;
    g_kickedThisStretch = true;
    if (sehWrite32(at, 5))
        Log::get().note("onfoot stereo: on foot per Status.json for %.1f s and the game still asks for mode 3; asked "
                        "for its on-foot mode (5) in its place (the load-in kick).",
                        since / 1000.0);
}

void InstallStereoPatch() {
    g_stereoTried = true;
    if (!g_base) g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    if (const char* why = checkTarget(g_base)) {
        Log::get().note("onfoot stereo: not installed: %s.", why);
        return;
    }
    const uintptr_t site = g_base + kStoreRva;
    if (std::memcmp(reinterpret_cast<const void*>(site - 3), kStoreBytes, sizeof(kStoreBytes)) != 0) {
        Log::get().note("onfoot stereo: not installed: the display-mode store's bytes are not the expected ones.");
        return;
    }
    uint8_t* stub = allocateRelay(site);
    uint8_t* data = stub ? allocateRelay(site) : nullptr;
    if (!stub || !data) {
        Log::get().note("onfoot stereo: not installed: no memory near the game's code.");
        return;
    }
    const uintptr_t st = reinterpret_cast<uintptr_t>(stub), dt = reinterpret_cast<uintptr_t>(data);
    // rsi is the function's own object: [rsi+7F8h] the one whose +20h holds
    // the requested mode (the load-in kick reads and writes it), [rsi+50h] P.
    uint8_t code[0x2B] = {
        0x48, 0x89, 0x35, 0, 0, 0, 0,  // 00 mov [rip+rsi],rsi
        0x80, 0x3D, 0, 0, 0, 0, 0x00,  // 07 cmp byte ptr [rip+flag],0
        0x74, 0x10,                    // 0E je 20
        0x83, 0xF8, 0x05,              // 10 cmp eax,5
        0x75, 0x0B,                    // 13 jne 20
        0xB8, 0x03, 0x00, 0x00, 0x00,  // 15 mov eax,3
        0xFF, 0x05, 0, 0, 0, 0,        // 1A inc dword ptr [rip+count]
        0x89, 0x81, 0x98, 0x36, 0x00, 0x00,  // 20 mov [rcx+3698h],eax
        0xE9, 0, 0, 0, 0};             // 26 jmp back
    const int32_t rsiDisp = static_cast<int32_t>(static_cast<intptr_t>(dt + 8) - static_cast<intptr_t>(st + 0x07));
    const int32_t flagDisp = static_cast<int32_t>(static_cast<intptr_t>(dt) - static_cast<intptr_t>(st + 0x0E));
    const int32_t countDisp = static_cast<int32_t>(static_cast<intptr_t>(dt + 4) - static_cast<intptr_t>(st + 0x20));
    const int32_t backDisp = static_cast<int32_t>(static_cast<intptr_t>(site + 6) - static_cast<intptr_t>(st + 0x2B));
    std::memcpy(code + 0x03, &rsiDisp, 4);
    std::memcpy(code + 0x09, &flagDisp, 4);
    std::memcpy(code + 0x1C, &countDisp, 4);
    std::memcpy(code + 0x27, &backDisp, 4);
    std::memcpy(stub, code, sizeof(code));
    DWORD old = 0;
    if (!VirtualProtect(stub, 4096, PAGE_EXECUTE_READ, &old) ||
        !FlushInstructionCache(GetCurrentProcess(), stub, sizeof(code))) {
        Log::get().note("onfoot stereo: not installed: the stub could not be made executable.");
        return;
    }
    g_stereoStub = stub;
    g_stereoData = data;
    // The site as one aligned 8-byte store: +0x281B050 keeps its two bytes
    // (the tail of the mov eax before it), then E9 rel32 to the stub and a
    // NOP over the store's last byte. A thread can only be at an instruction
    // boundary, and +0x281B04F's bytes are unchanged, so none sees half.
    const uintptr_t q = site - 2;
    if (q % 8 != 0) {
        Log::get().note("onfoot stereo: not installed: the site is not where an atomic store can cover it.");
        return;
    }
    uint8_t bytes[8];
    std::memcpy(bytes, reinterpret_cast<const void*>(q), 8);
    const int32_t toStub = static_cast<int32_t>(static_cast<intptr_t>(st) - static_cast<intptr_t>(site + 5));
    bytes[2] = 0xE9;
    std::memcpy(bytes + 3, &toStub, 4);
    bytes[7] = 0x90;
    int64_t value;
    std::memcpy(&value, bytes, 8);
    if (!VirtualProtect(reinterpret_cast<void*>(q), 8, PAGE_EXECUTE_READWRITE, &old)) {
        Log::get().note("onfoot stereo: not installed: the game's code page could not be made writable.");
        return;
    }
    InterlockedExchange64(reinterpret_cast<volatile LONG64*>(q), value);
    DWORD ignored = 0;
    VirtualProtect(reinterpret_cast<void*>(q), 8, old, &ignored);
    FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(q), 8);
    g_stereoPatched = true;
    Log::get().note("onfoot stereo: installed at the display-mode store (+0x%llX): on foot the engine is kept in "
                    "HMD stereo (mode 3) instead of HMD Cinema (5).",
                    static_cast<unsigned long long>(kStoreRva));
}

void StereoFrame() {
    if (!g_stereoPatched) {
        setEyeSwap(false);
        return;
    }
    const uint32_t count = *reinterpret_cast<volatile const uint32_t*>(g_stereoData + 4);
    if (count && !g_stereoLastCount)
        Log::get().note("onfoot stereo: the game asked for HMD Cinema (5); kept at HMD stereo (3).");
    const bool substituted = count != g_stereoLastCount;
    if (substituted) g_quietFrames = 0;
    else if (g_quietFrames != ~0u) ++g_quietFrames;
    g_stereoLastCount = count;
    KickFrame(substituted);
    // Each image to the other eye (onfoot_stereo_swap_eyes) while on foot.
    setEyeSwap(g_swapWanted && onFootStereoWanted());
}

}  // namespace

bool onFootStereoHolding() { return g_stereoPatched && g_stereoData && *g_stereoData; }

// A few frames of grace, so one frame the store did not run is not a trip
// to the ship and back.
bool onFootStereoWanted() { return onFootStereoHolding() && g_quietFrames < 30; }

void stereoModeProbeConfigure(Config& cfg) {
    const bool stereo = cfg.getBool("experimental.onfoot_stereo", false);
    if (stereo && !g_stereoTried) InstallStereoPatch();
    if (g_stereoData) {
        const uint8_t want = stereo ? 1 : 0;
        if (*g_stereoData != want)
            Log::get().note(stereo ? "onfoot stereo: ON (live)." : "onfoot stereo: off (live; the game's own mode).");
        *g_stereoData = want;
    }
    g_watchWanted = cfg.getBool("experimental.stereo_mode_watch", false);
    const bool kick = cfg.getBool("experimental.onfoot_stereo_kick", true);
    if (kick != g_kickWanted) Log::get().note("onfoot stereo: load-in kick %s.", kick ? "on" : "off");
    g_kickWanted = kick;
    const bool swap = cfg.getBool("experimental.onfoot_stereo_swap_eyes", false);
    if (swap != g_swapWanted)
        Log::get().note("onfoot stereo: eyes %s on foot (live).", swap ? "SWAPPED" : "as the game submits them");
    g_swapWanted = swap;
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
