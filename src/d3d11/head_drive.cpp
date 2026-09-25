#include "head_drive.h"

#include <windows.h>
#include <tlhelp32.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "../common/config.h"
#include "../common/frame_flag.h"
#include "../common/log.h"

namespace edvr {

namespace {

// Build 332841 (stereo_mode_probe.cpp's identity pair).
constexpr uint32_t kExpectedTimestamp = 1788384820u;
constexpr uint32_t kExpectedImageSize = 104894464u;
constexpr uintptr_t kClampRva = 0x1A9CB11u;
constexpr uint8_t kClampBytes[30] = {0x41, 0x0F, 0x2F, 0xC0,                          // comiss xmm0,xmm8
                                     0x77, 0x08,                                      // ja +8
                                     0x0F, 0x28, 0xC6,                                // movaps xmm0,xmm6
                                     0xF3, 0x41, 0x0F, 0x5D, 0xC0,                    // minss xmm0,xmm8
                                     0xF3, 0x0F, 0x11, 0x87, 0x1C, 0x07, 0x00, 0x00,  // movss [rdi+71Ch],xmm0
                                     0xF3, 0x0F, 0x11, 0x87, 0x18, 0x07, 0x00, 0x00};  // movss [rdi+718h],xmm0
constexpr uintptr_t kBodyRows = 0x5B0;  // right, up, forward (float4 rows)
constexpr uintptr_t kLeadYaw = 0x724, kRecoilYaw = 0x734;  // the camera's yaw off the body (radians)
constexpr uintptr_t kPitch = 0x718, kRecoilPitch = 0x738;  // the camera's pitch: [0x718] - [0x738] (positive down)

uintptr_t g_base = 0;
bool g_tried = false, g_installed = false;
std::atomic<bool> g_want{false};
volatile uint8_t* g_flag = nullptr;  // the stub's gate, in the stub's page

// --- the game thread's side --------------------------------------------------

std::atomic<uintptr_t> g_player{0};
ULONGLONG g_playerMs = 0;
double g_yawUsed = 0;
bool g_haveYaw = false;
std::atomic<uint64_t> g_driven{0}, g_others{0}, g_noPose{0}, g_adopted{0};
std::atomic<ULONGLONG> g_drivenMs{0};  // the last game frame the head drove

// A game frame's record: the head yaw it applied, the body frame it left,
// and the camera's yaw off that body (Y+0x724, the stick turn's lead, plus
// Y+0x734, the recoil's; positive right, as the look's vfC8 applies it).
struct Record {
    volatile LONG seq;  // odd while written
    float yaw, camYaw, lead;  // lead: Y+0x724 alone
    float right[3], up[3], fwd[3];
};
constexpr int kRecords = 32;
Record g_records[kRecords];
volatile LONG g_recordHead = 0;
volatile LONG g_recordFloor = 0;  // the first record of the component driven now

__declspec(noinline) bool SehRead(uintptr_t at, void* out, size_t n) noexcept {
    __try {
        std::memcpy(out, reinterpret_cast<const void*>(at), n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

__declspec(noinline) bool SehWrite(uintptr_t at, const void* in, size_t n) noexcept {
    __try {
        std::memcpy(reinterpret_cast<void*>(at), in, n);
        return true;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
}

void Push(float yaw, float camYaw, float lead, const float* rows) {
    const LONG i = InterlockedIncrement(&g_recordHead) - 1;
    Record& r = g_records[i % kRecords];
    InterlockedIncrement(&r.seq);  // odd: being written
    r.yaw = yaw;
    r.camYaw = camYaw;
    r.lead = lead;
    for (int k = 0; k < 3; ++k) {
        r.right[k] = rows[k];
        r.up[k] = rows[4 + k];
        r.fwd[k] = rows[8 + k];
    }
    InterlockedIncrement(&r.seq);
}

double Wrap(double a) {
    constexpr double kPi = 3.14159265358979323846;
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
}

// The head's yaw and pitch in the head look's form (onfoot_look.cpp's
// TakeHeadRotation: Q = C R C, C = diag(1, 1, -1); yaw atan2(q02, q22),
// pitch asin(-q12), positive down). False for no pose or a torn read.
bool HeadYawPitch(double* yaw, double* pitch) {
    float m[12];
    if (!headPose(m)) return false;
    for (int i = 0; i < 3; ++i) {
        const double l = double(m[4 * i]) * m[4 * i] + double(m[4 * i + 1]) * m[4 * i + 1] +
                         double(m[4 * i + 2]) * m[4 * i + 2];
        if (std::fabs(l - 1) > 1e-3) return false;
    }
    // q[i][j] = m[i][j] s[i] s[j], s = (1, 1, -1).
    const double q02 = -double(m[2]), q22 = m[10], q12 = -double(m[6]);
    *yaw = std::atan2(q02, q22);
    *pitch = std::asin(std::fmax(-1.0, std::fmin(1.0, -q12)));
    return true;
}

// Which component is the player's: some 22 NPCs run the same input update,
// and the player's goes quiet aboard a ship (flight of 2026-09-24: taking the
// next caller after a quiet half second drove an NPC after disembarking).
// While the render side seeks, every component's camera as the game left it
// goes here; the render side picks the one the drawn camera matches, and
// that component is adopted at its next call.
struct Candidate {
    volatile LONG seq;  // odd while written
    uintptr_t y;
    ULONGLONG ms;
    float heading[3], up[3], pitch;
};
constexpr int kCandidates = 256;
Candidate g_candidates[kCandidates];
std::atomic<bool> g_seeking{true};
std::atomic<uintptr_t> g_pick{0};

void NoteCandidate(uintptr_t y, ULONGLONG now) noexcept {
    float rows[12], lead = 0, recoil = 0, pitch = 0, recoilPitch = 0;
    if (!SehRead(y + kBodyRows, rows, sizeof(rows))) return;
    SehRead(y + kLeadYaw, &lead, 4);
    SehRead(y + kRecoilYaw, &recoil, 4);
    SehRead(y + kPitch, &pitch, 4);
    SehRead(y + kRecoilPitch, &recoilPitch, 4);
    const double c = std::cos(lead + recoil), s = std::sin(lead + recoil);
    Candidate& k = g_candidates[((y >> 7) ^ (y >> 17)) % kCandidates];
    InterlockedIncrement(&k.seq);
    k.y = y;
    k.ms = now;
    for (int i = 0; i < 3; ++i) {
        k.heading[i] = static_cast<float>(rows[8 + i] * c + rows[i] * s);
        k.up[i] = rows[4 + i];
    }
    k.pitch = pitch - recoilPitch;
    InterlockedIncrement(&k.seq);
}

// Called by the stub on the game's thread, once a frame per look component
// that runs the input update, with rdi (the component) and the candidate
// pitch (in: the stick's; out: the head's). True: use *pitch.
bool GameHook(uintptr_t y, float* pitch) noexcept {
    if (!g_want.load(std::memory_order_relaxed)) return false;
    const ULONGLONG now = GetTickCount64();
    const uintptr_t player = g_player.load(std::memory_order_relaxed);
    if (g_seeking.load(std::memory_order_relaxed)) NoteCandidate(y, now);
    if (y != player) {
        // Another component: ours only when the render side picked it.
        if (y != g_pick.load(std::memory_order_relaxed)) {
            g_others.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        g_recordFloor = g_recordHead;
        g_player.store(y, std::memory_order_relaxed);
        g_pick.store(0, std::memory_order_relaxed);
        g_seeking.store(false, std::memory_order_relaxed);
        g_adopted.fetch_add(1, std::memory_order_relaxed);
        g_haveYaw = false;
    }
    if (now - g_playerMs > 500) g_haveYaw = false;  // back after a pause: no turn for the gap
    g_playerMs = now;
    double yaw = 0, headPitch = 0;
    if (!HeadYawPitch(&yaw, &headPitch)) {
        g_noPose.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    float rows[12], camYaw[2] = {};
    if (!SehRead(y + kBodyRows, rows, sizeof(rows))) return false;
    SehRead(y + kLeadYaw, &camYaw[0], 4);
    SehRead(y + kRecoilYaw, &camYaw[1], 4);
    // The last frame's record: the yaw it applied, the frame it left.
    if (g_haveYaw) Push(static_cast<float>(g_yawUsed), camYaw[0] + camYaw[1], camYaw[0], rows);
    // The body turned by the head's yaw since, about its up row.
    const double d = g_haveYaw ? Wrap(yaw - g_yawUsed) : 0.0;
    if (d != 0.0) {
        const double c = std::cos(d), s = std::sin(d);
        float turned[12];
        std::memcpy(turned, rows, sizeof(turned));
        for (int k = 0; k < 3; ++k) {
            turned[k] = static_cast<float>(rows[k] * c - rows[8 + k] * s);      // right
            turned[8 + k] = static_cast<float>(rows[8 + k] * c + rows[k] * s);  // forward
        }
        SehWrite(y + kBodyRows, turned, 12);
        SehWrite(y + kBodyRows + 32, turned + 8, 12);
    }
    g_yawUsed = yaw;
    g_haveYaw = true;
    *pitch = static_cast<float>(headPitch);
    g_driven.fetch_add(1, std::memory_order_relaxed);
    g_drivenMs.store(now, std::memory_order_relaxed);
    return true;
}

// --- installing -------------------------------------------------------------

__declspec(noinline) const char* CheckTarget(uintptr_t base) noexcept {
    __try {
        uint32_t peOff = 0, timestamp = 0, imageSize = 0;
        std::memcpy(&peOff, reinterpret_cast<const void*>(base + 0x3C), 4);
        if (peOff > 0x1000) return "PE header offset implausible";
        std::memcpy(&timestamp, reinterpret_cast<const void*>(base + peOff + 8), 4);
        std::memcpy(&imageSize, reinterpret_cast<const void*>(base + peOff + 0x50), 4);
        if (timestamp != kExpectedTimestamp || imageSize != kExpectedImageSize)
            return "not build 332841 (PE timestamp/size mismatch)";
        const uint8_t* site = reinterpret_cast<const uint8_t*>(base + kClampRva);
        if (std::memcmp(site, kClampBytes, sizeof(kClampBytes)) != 0)
            return "the pitch clamp's bytes are not the expected ones";
        if (site[-1] != 0x00 || site[6] != 0x0F) return "the bytes around the pitch clamp are not the expected ones";
        return nullptr;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return "a read faulted while checking the executable";
    }
}

uint8_t* AllocateNear(uintptr_t target) noexcept {
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

// One aligned 8-byte store into the game's code with every other thread
// stopped, none of them inside (lo, hi) -- where the old instructions'
// boundaries no longer are. Retries a few times. Nothing that could take a
// lock runs while they are stopped.
bool PatchStopped(uintptr_t q, int64_t value, uintptr_t lo, uintptr_t hi) {
    constexpr int kMaxThreads = 1024;
    static HANDLE threads[kMaxThreads];
    const DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    for (int attempt = 0; attempt < 20; ++attempt) {
        int n = 0;
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
        if (snap == INVALID_HANDLE_VALUE) return false;
        THREADENTRY32 te{};
        te.dwSize = sizeof(te);
        if (Thread32First(snap, &te)) {
            do {
                if (te.th32OwnerProcessID != pid || te.th32ThreadID == self || n >= kMaxThreads) continue;
                HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT, FALSE, te.th32ThreadID);
                if (!h) continue;
                if (SuspendThread(h) == static_cast<DWORD>(-1)) {
                    CloseHandle(h);
                    continue;
                }
                threads[n++] = h;
            } while (Thread32Next(snap, &te));
        }
        CloseHandle(snap);
        bool clear = true;
        for (int i = 0; i < n && clear; ++i) {
            CONTEXT ctx{};
            ctx.ContextFlags = CONTEXT_CONTROL;
            if (GetThreadContext(threads[i], &ctx) && ctx.Rip > lo && ctx.Rip < hi) clear = false;
        }
        bool done = false;
        if (clear) {
            DWORD old = 0;
            if (VirtualProtect(reinterpret_cast<void*>(q), 8, PAGE_EXECUTE_READWRITE, &old)) {
                InterlockedExchange64(reinterpret_cast<volatile LONG64*>(q), value);
                DWORD ignored = 0;
                VirtualProtect(reinterpret_cast<void*>(q), 8, old, &ignored);
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<void*>(q), 8);
                done = true;
            }
        }
        for (int i = 0; i < n; ++i) {
            ResumeThread(threads[i]);
            CloseHandle(threads[i]);
        }
        if (done) return true;
        if (!clear) {
            Sleep(2);
            continue;
        }
        return false;
    }
    return false;
}

void Install() {
    g_tried = true;
    if (!g_base) g_base = reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr));
    if (const char* why = CheckTarget(g_base)) {
        Log::get().note("onfoot head drive: not installed: %s.", why);
        return;
    }
    const uintptr_t site = g_base + kClampRva;
    uint8_t* stub = AllocateNear(site);
    if (!stub) {
        Log::get().note("onfoot head drive: not installed: no memory near the game's code.");
        return;
    }
    const uintptr_t st = reinterpret_cast<uintptr_t>(stub);
    uint8_t* flag = stub + 0x100;
    // Gate; save the volatile registers; GameHook(rdi, &xmm8's slot); xmm8
    // from the slot when it says so; restore; then the replaced comiss/ja,
    // back into the game's code. rsp is 16-aligned at the site (a call just
    // returned there): 7 pushes and 0x88 keep it aligned for the call.
    uint8_t code[0xAB] = {
        0x80, 0x3D, 0, 0, 0, 0, 0x00,                    // 00 cmp byte ptr [rip+flag],0
        0x0F, 0x84, 0x8F, 0x00, 0x00, 0x00,              // 07 je 9C
        0x50, 0x51, 0x52,                                // 0D push rax, rcx, rdx
        0x41, 0x50, 0x41, 0x51, 0x41, 0x52, 0x41, 0x53,  // 10 push r8..r11
        0x48, 0x81, 0xEC, 0x88, 0x00, 0x00, 0x00,        // 18 sub rsp,88h
        0x0F, 0x11, 0x44, 0x24, 0x20,                    // 1F movups [rsp+20h],xmm0
        0x0F, 0x11, 0x4C, 0x24, 0x30,                    // 24 movups [rsp+30h],xmm1
        0x0F, 0x11, 0x54, 0x24, 0x40,                    // 29 movups [rsp+40h],xmm2
        0x0F, 0x11, 0x5C, 0x24, 0x50,                    // 2E movups [rsp+50h],xmm3
        0x0F, 0x11, 0x64, 0x24, 0x60,                    // 33 movups [rsp+60h],xmm4
        0x0F, 0x11, 0x6C, 0x24, 0x70,                    // 38 movups [rsp+70h],xmm5
        0xF3, 0x44, 0x0F, 0x11, 0x84, 0x24, 0x80, 0x00, 0x00, 0x00,  // 3D movss [rsp+80h],xmm8
        0x48, 0x89, 0xF9,                                            // 47 mov rcx,rdi
        0x48, 0x8D, 0x94, 0x24, 0x80, 0x00, 0x00, 0x00,              // 4A lea rdx,[rsp+80h]
        0x48, 0xB8, 0, 0, 0, 0, 0, 0, 0, 0,                          // 52 mov rax,GameHook
        0xFF, 0xD0,                                                  // 5C call rax
        0x84, 0xC0,                                                  // 5E test al,al
        0x74, 0x0A,                                                  // 60 je 6C
        0xF3, 0x44, 0x0F, 0x10, 0x84, 0x24, 0x80, 0x00, 0x00, 0x00,  // 62 movss xmm8,[rsp+80h]
        0x0F, 0x10, 0x44, 0x24, 0x20,                                // 6C movups xmm0,[rsp+20h]
        0x0F, 0x10, 0x4C, 0x24, 0x30,                                // 71 movups xmm1,[rsp+30h]
        0x0F, 0x10, 0x54, 0x24, 0x40,                                // 76 movups xmm2,[rsp+40h]
        0x0F, 0x10, 0x5C, 0x24, 0x50,                                // 7B movups xmm3,[rsp+50h]
        0x0F, 0x10, 0x64, 0x24, 0x60,                                // 80 movups xmm4,[rsp+60h]
        0x0F, 0x10, 0x6C, 0x24, 0x70,                                // 85 movups xmm5,[rsp+70h]
        0x48, 0x81, 0xC4, 0x88, 0x00, 0x00, 0x00,                    // 8A add rsp,88h
        0x41, 0x5B, 0x41, 0x5A, 0x41, 0x59, 0x41, 0x58,              // 91 pop r11..r8
        0x5A, 0x59, 0x58,                                            // 99 pop rdx, rcx, rax
        0x41, 0x0F, 0x2F, 0xC0,                                      // 9C comiss xmm0,xmm8
        0x0F, 0x87, 0, 0, 0, 0,                                      // A0 ja site+0Eh
        0xE9, 0, 0, 0, 0};                                           // A6 jmp site+06h
    auto rel = [](uintptr_t to, uintptr_t next) {
        return static_cast<int32_t>(static_cast<intptr_t>(to) - static_cast<intptr_t>(next));
    };
    const int32_t flagDisp = rel(reinterpret_cast<uintptr_t>(flag), st + 0x07);
    const uint64_t hook = reinterpret_cast<uint64_t>(&GameHook);
    const int32_t jaDisp = rel(site + 0x0E, st + 0xA6);
    const int32_t backDisp = rel(site + 0x06, st + 0xAB);
    std::memcpy(code + 0x02, &flagDisp, 4);
    std::memcpy(code + 0x54, &hook, 8);
    std::memcpy(code + 0xA2, &jaDisp, 4);
    std::memcpy(code + 0xA7, &backDisp, 4);
    std::memcpy(stub, code, sizeof(code));
    *flag = g_want.load() ? 1 : 0;
    DWORD old = 0;
    if (!VirtualProtect(stub, 4096, PAGE_EXECUTE_READWRITE, &old) ||
        !FlushInstructionCache(GetCurrentProcess(), stub, sizeof(code))) {
        Log::get().note("onfoot head drive: not installed: the stub could not be made executable.");
        return;
    }
    g_flag = flag;
    // The site as one aligned 8-byte store: +0x1A9CB10 keeps its byte (the
    // call's last), E9 rel32 over the comiss, a NOP over the ja's second
    // byte, +0x1A9CB17 (movaps) untouched -- with no thread stopped at the
    // ja (+0x1A9CB15), the one boundary that moves.
    const uintptr_t q = site - 1;
    if (q % 8 != 0) {
        Log::get().note("onfoot head drive: not installed: the site is not where an atomic store can cover it.");
        return;
    }
    uint8_t bytes[8];
    std::memcpy(bytes, reinterpret_cast<const void*>(q), 8);
    const int32_t toStub = rel(st, site + 5);
    bytes[1] = 0xE9;
    std::memcpy(bytes + 2, &toStub, 4);
    bytes[6] = 0x90;
    int64_t value;
    std::memcpy(&value, bytes, 8);
    if (!PatchStopped(q, value, site, site + 6)) {
        Log::get().note("onfoot head drive: not installed: the game's code could not be patched.");
        return;
    }
    g_installed = true;
    Log::get().note("onfoot head drive: installed at the look's pitch clamp (+0x%llX): the head's pitch goes into "
                    "the game's look, its yaw turns the body.",
                    static_cast<unsigned long long>(kClampRva));
}

// --- the render side --------------------------------------------------------

std::atomic<uint64_t> g_matched{0}, g_unmatched{0};
double g_residualSum = 0;
uint64_t g_residualCount = 0;
double g_residualMax = 0;
ULONGLONG g_reportMs = 0;

}  // namespace

void headDriveConfigure(Config& cfg) {
    const bool want = cfg.getBool("experimental.onfoot_head_drive", false);
    if (want != g_want.load())
        Log::get().note(want ? "onfoot head drive: ON (live): the head moves the game's own look."
                             : "onfoot head drive: off (live): the stick has the look again.");
    g_want.store(want);
    if (want && !g_tried) Install();
    if (g_flag) *g_flag = want ? 1 : 0;
}

// The drawn forward's heading against a record's camera (its body turned by
// its camera yaw): the cosine between them, and the heading's angle off the
// record's body (positive right).
bool Heading(const Record& c, const double* f, double* match, double* offBody) {
    const double fu = f[0] * c.up[0] + f[1] * c.up[1] + f[2] * c.up[2];
    double h[3] = {f[0] - fu * c.up[0], f[1] - fu * c.up[1], f[2] - fu * c.up[2]};
    const double hl = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    if (hl < 1e-3) return false;
    for (double& v : h) v /= hl;
    const double hf = h[0] * c.fwd[0] + h[1] * c.fwd[1] + h[2] * c.fwd[2];
    const double hr = h[0] * c.right[0] + h[1] * c.right[1] + h[2] * c.right[2];
    const double cy = std::cos(c.camYaw), sy = std::sin(c.camYaw);
    *match = hf * cy + hr * sy;  // h . (fwd cos + right sin)
    *offBody = std::atan2(hr, hf);
    return true;
}

Record g_lastMatch{};
bool g_haveLastMatch = false;

// Components let go (their camera was not the drawn one): not picked again
// for half a minute.
struct Refused {
    uintptr_t y;
    ULONGLONG ms;
};
Refused g_refused[8] = {};
int g_refusedNext = 0;
uintptr_t g_seekBest = 0;
uint64_t g_seekCalls = 0, g_seekWithin = 0;
double g_seekNearestErr = 1e9, g_seekNearestH = 0, g_seekNearestP = 0;
bool g_ambiguousNoted = false;
int g_seekRun = 0;

// The component whose camera, as the game left it, the drawn camera is: the
// clear nearest within two degrees in heading and in pitch, the same one for a
// quarter second of frames. (Flight of 2026-09-24 23:58: at three degrees
// of heading and any pitch, eight NPCs were picked and driven before the
// player's, which matched to 0.02 and 0.01 degrees.)
void Seek(const double* f) {
    g_seeking.store(true, std::memory_order_relaxed);
    const ULONGLONG now = GetTickCount64();
    double bestErr = 1e9, secondErr = 1e9, bestH = 0, bestP = 0, nearestErr = 1e9, nearestH = 0, nearestP = 0;
    uintptr_t best = 0;
    int within = 0;
    ++g_seekCalls;
    for (const Candidate& r : g_candidates) {
        const LONG s0 = r.seq;
        if (s0 & 1) continue;
        Candidate c;
        std::memcpy(&c, &r, sizeof(c));
        if (r.seq != s0 || !c.y || now - c.ms > 250) continue;
        const double fu = f[0] * c.up[0] + f[1] * c.up[1] + f[2] * c.up[2];
        double h[3] = {f[0] - fu * c.up[0], f[1] - fu * c.up[1], f[2] - fu * c.up[2]};
        const double hl = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
        if (hl < 1e-3) continue;
        const double m = (h[0] * c.heading[0] + h[1] * c.heading[1] + h[2] * c.heading[2]) / hl;
        const double headingErr = std::acos(std::fmax(-1.0, std::fmin(1.0, m)));
        const double pitchErr = std::fabs(-std::asin(std::fmax(-1.0, std::fmin(1.0, fu))) - c.pitch);
        if (headingErr + pitchErr < nearestErr) {
            nearestErr = headingErr + pitchErr;
            nearestH = headingErr;
            nearestP = pitchErr;
        }
        if (headingErr > 0.0349 || pitchErr > 0.0349) continue;  // 2 degrees
        bool refused = false;
        for (const Refused& x : g_refused) refused |= x.y == c.y && now - x.ms < 30000;
        if (refused) continue;
        ++within;
        if (headingErr + pitchErr < bestErr) {
            secondErr = bestErr;
            bestErr = headingErr + pitchErr;
            best = c.y;
            bestH = headingErr;
            bestP = pitchErr;
        } else if (headingErr + pitchErr < secondErr) {
            secondErr = headingErr + pitchErr;
        }
    }
    if (nearestErr < g_seekNearestErr) {
        g_seekNearestErr = nearestErr;
        g_seekNearestH = nearestH;
        g_seekNearestP = nearestP;
    }
    if (within) ++g_seekWithin;
    // Several within two degrees: only a clear nearest (half the next one's
    // error at most); the half second of unmatched frames still lets a wrong
    // one go, and it is not picked again for 30 s.
    if (within > 1 && !(bestErr <= 0.5 * secondErr)) best = 0;
    // More than one within a degree (flight of 2026-09-25 02:10: never
    // picked in eight minutes): the nearest all the same -- the half second
    // of unmatched frames lets a wrong one go, and it is not picked again.
    if (within > 1 && !g_ambiguousNoted) {
        g_ambiguousNoted = true;
        Log::get().note("onfoot head drive: %d components' cameras within a degree of the drawn one; the nearest (%p, "
                        "heading off %.2f, pitch off %.2f degrees) is tried first.",
                        within, reinterpret_cast<void*>(best), bestH * 57.29578, bestP * 57.29578);
    }
    g_seekRun = best && best == g_seekBest ? g_seekRun + 1 : 0;
    g_seekBest = best;
    if (!best || g_seekRun < 20 || g_pick.load(std::memory_order_relaxed) == best) return;
    g_pick.store(best, std::memory_order_relaxed);
    Log::get().note("onfoot head drive: picked component %p, the drawn camera's (heading off %.2f, pitch off %.2f degrees)",
                    reinterpret_cast<void*>(best), bestH * 57.29578, bestP * 57.29578);
}

int g_unmatchedRun = 0;
uintptr_t g_seenPlayer = 0;
std::atomic<bool> g_lost{false};

bool headDriveCamera(const double axes[3][3], double qGame[3][3], bool early) {
    if (!g_installed || !g_want.load(std::memory_order_relaxed)) return false;
    const double* f = axes[2];
    uintptr_t player = g_player.load(std::memory_order_relaxed);
    if (player != g_seenPlayer) {
        g_seenPlayer = player;
        g_unmatchedRun = 0;
        g_haveLastMatch = false;
        g_lost.store(false, std::memory_order_relaxed);
    }
    // Half a second of drawn frames none of its records match: the drawn
    // camera is not the one driven. Let it go and seek again.
    if (player && g_unmatchedRun >= 45) {
        Log::get().note("onfoot head drive: the drawn camera is not component %p's; seeking",
                        reinterpret_cast<void*>(player));
        g_lost.store(true, std::memory_order_relaxed);
        g_refused[g_refusedNext++ % 8] = {player, GetTickCount64()};
        g_seekRun = 0;
        g_player.store(0, std::memory_order_relaxed);
        player = g_seenPlayer = 0;
        g_unmatchedRun = 0;
        g_haveLastMatch = false;
    }
    if (!player) {
        if (!early) Seek(f);
        return false;
    }
    const LONG head = g_recordHead, floor = g_recordFloor;
    double best = -2;
    Record bestRecord{};
    for (int k = 0; k < kRecords && k < head - floor; ++k) {
        const Record& r = g_records[(head - 1 - k) % kRecords];
        const LONG s0 = r.seq;
        if (s0 & 1) continue;
        Record c;
        std::memcpy(&c, &r, sizeof(c));
        if (r.seq != s0) continue;
        double match = 0, off = 0;
        if (Heading(c, f, &match, &off) && match > best) {
            best = match;
            bestRecord = c;
        }
    }
    // Within 3 degrees of a frame's camera: that frame's head sample. Else
    // the last match's (its body's turn since is the error: a stick turn in
    // the gap); with none yet, no residual at all -- never the whole head
    // on top of a camera that already has it.
    if (early && best < 0.9986) return false;
    if (best >= 0.9986) {
        g_matched.fetch_add(1, std::memory_order_relaxed);
        g_unmatchedRun = 0;
        g_lastMatch = bestRecord;
        g_haveLastMatch = true;
    } else {
        g_unmatched.fetch_add(1, std::memory_order_relaxed);
        ++g_unmatchedRun;
        if (!g_haveLastMatch) return false;
        bestRecord = g_lastMatch;
    }
    double match = 0, offBody = 0;
    if (!Heading(bestRecord, f, &match, &offBody)) return false;
    // The stick's lead is a turn of the world, not of the camera in it: on
    // the ground the stick turns the camera off the body (Y+0x724) and the
    // body follows a second later (flight of 2026-09-24: in ADS the crosshair
    // slid, the world held, then snapped). Taken out of the camera's yaw, the
    // world turns with the stick at once and the body's catch-up is no turn.
    // The recoil's yaw stays in: the crosshair kicks, not the world.
    const double psi = bestRecord.yaw + offBody - bestRecord.lead;
    const double bestUp[3] = {bestRecord.up[0], bestRecord.up[1], bestRecord.up[2]};
    const double fu = f[0] * bestUp[0] + f[1] * bestUp[1] + f[2] * bestUp[2];
    const double b = -std::asin(std::fmax(-1.0, std::fmin(1.0, fu)));  // positive down
    // Ry(psi) Rx(b).
    const double cy = std::cos(psi), sy = std::sin(psi), cb = std::cos(b), sb = std::sin(b);
    const double ry[3][3] = {{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}};
    const double rx[3][3] = {{1, 0, 0}, {0, cb, -sb}, {0, sb, cb}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) qGame[i][j] = ry[i][0] * rx[0][j] + ry[i][1] * rx[1][j] + ry[i][2] * rx[2][j];
    return true;
}

bool headDriveActive() {
    return g_installed && g_want.load(std::memory_order_relaxed) && g_player.load(std::memory_order_relaxed) &&
           GetTickCount64() - g_drivenMs.load(std::memory_order_relaxed) < 500;
}

void headDriveNoteResidual(double degrees) {
    g_residualSum += degrees;
    ++g_residualCount;
    if (degrees > g_residualMax) g_residualMax = degrees;
}

void headDriveFrame() {
    if (!g_installed) return;
    const ULONGLONG now = GetTickCount64();
    if (!g_reportMs) g_reportMs = now;
    if (now - g_reportMs < 10000) return;
    g_reportMs = now;
    const uint64_t driven = g_driven.exchange(0), others = g_others.exchange(0), noPose = g_noPose.exchange(0);
    const uint64_t matched = g_matched.exchange(0), unmatched = g_unmatched.exchange(0);
    if (!driven && !matched && !unmatched && !others) return;
    Log::get().note("onfoot head drive: last 10 s %llu game frames driven (component %p, %llu adopted; %llu calls "
                    "from other components, %llu without a head pose); drawn frames matched to their head sample "
                    "%llu, unmatched %llu; residual turn mean %.2f max %.2f degrees.",
                    static_cast<unsigned long long>(driven), reinterpret_cast<void*>(g_player.load()),
                    static_cast<unsigned long long>(g_adopted.load()), static_cast<unsigned long long>(others),
                    static_cast<unsigned long long>(noPose), static_cast<unsigned long long>(matched),
                    static_cast<unsigned long long>(unmatched),
                    g_residualCount ? g_residualSum / double(g_residualCount) : 0.0, g_residualMax);
    g_residualSum = 0;
    g_residualCount = 0;
    g_residualMax = 0;
    if (g_seekCalls)
        Log::get().note("onfoot head drive: seeking the player's camera: %llu frames, %llu with a component within two "
                        "degrees; the nearest seen was %.2f off in heading, %.2f in pitch.",
                        static_cast<unsigned long long>(g_seekCalls), static_cast<unsigned long long>(g_seekWithin),
                        g_seekNearestH * 57.29578, g_seekNearestP * 57.29578);
    g_seekCalls = g_seekWithin = 0;
    g_seekNearestErr = 1e9;
}

}  // namespace edvr
