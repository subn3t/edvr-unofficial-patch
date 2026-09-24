#include "onfoot_look.h"

#include <windows.h>
#include <d3dcompiler.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "../common/config.h"
#include "../common/frame_flag.h"
#include "../common/guard.h"
#include "../common/log.h"
#include "binding_shadow.h"
#include "draw_census.h"
#include "journal_watch.h"
#include "vscreen.h"

namespace edvr {

namespace detail {
bool g_onFootLookEnabled = false;
bool g_onFootHeadLocked = false;
}  // namespace detail

namespace {

// Layout for build 332.841 (docs/cobra-onfoot-frame.md; the flat mod's
// docs/frame-capture-332841.md for the same registers in the cockpit).
constexpr UINT kViewOffset = 64;    // b0: the clip transform, 4 rows
constexpr UINT kViewMinBytes = kViewOffset + 64;
constexpr UINT kClipOffset = 4320;  // b1: cb1[270..273], the same transform by columns
// b1: last frame's camera-to-local [R_prev^T | pos], 3 rows. Left as the game
// wrote it (turning it by either frame's head showed no difference in the
// field); kept out of the scan, where a still camera's would pass for R^T.
constexpr UINT kPrevPoseOffset = 3728;
constexpr UINT kFrameMinBytes = kClipOffset + 64;

ID3D11Buffer* g_viewCb = nullptr;   // learned b0 (our reference, so the address is not reused)
ID3D11Buffer* g_frameCb = nullptr;  // learned b1 (ditto)
void* g_viewData = nullptr;         // mapped pointers, Map to Unmap
void* g_frameData = nullptr;
float g_viewShadow[16] = {};        // b0's clip rows as last written (after any turn: what the GPU got)
bool g_viewShadowFresh = false;
UINT g_viewBytes = 0, g_frameFloats = 0;

// The main view's projection: x/y scale (row lengths) and near, from the b0
// write the frame's first panel G-buffer draw used.
bool g_haveMain = false;
double g_mainScale[2] = {}, g_mainNear = 0;

// Per-frame learning, and whether the panel scene ran last frame (the turn
// is armed only then: in the cockpit the G-buffer is eye-sized).
bool g_foundThisFrame = false;
bool g_panelLastFrame = false;
uint32_t g_panelW = 0, g_panelH = 0;  // refreshed each frame

// The render-target check, cached by the binding shadow's generations.
uint32_t g_rtvGen = 0, g_dsvGen = 0;
bool g_rtvIsPanelGBuffer = false;
// The last R10G10B10A2 target with a depth bound, whatever its size: what the
// "on foot but no panel scene" line reports, so a size mismatch is visible.
uint32_t g_lastGbufW = 0, g_lastGbufH = 0;

// The head rotation for this frame's writes, in the game's view axes, taken
// once per frame at the first turn.
double g_q[3][3] = {};
double g_hRender[3][3] = {};  // the same pose's rotation in OpenVR axes, for the head-locked view's timewarp
bool g_qValid = false, g_qTaken = false;

// The frame's camera rotation as the game wrote it (rows right, up, forward),
// from the first main-view b1 write. Unusable when it is the identity: a
// view-space matrix would then pass for a copy of the camera.
double g_frameR[3][3] = {};
bool g_frameRValid = false, g_frameRUsable = false;

// Other buffers mapped for a whole rewrite while the panel scene runs: the
// candidates for camera copies, turned at their Unmap.
struct Pending {
    ID3D11Resource* res;
    float* data;
    UINT floats;
};
constexpr int kMaxPending = 32;
Pending g_pending[kMaxPending] = {};
constexpr UINT kMaxScanBytes = 16384;
// How close a copy has to be: the lighting passes compute their copies of the
// camera apart from the view slot's (up to 2e-4 off).
constexpr double kMatch = 1e-3;

// Developer switches (experimental.onfoot_head_look_skip): parts of the turn
// left out, to see by eye which one a visual fault follows. Live.
enum Part : unsigned { kPartView = 1, kPartOthers = 2, kPartMask = 4, kPartFrames = 8, kPartScaled = 16, kPartTimewarp = 32 };
unsigned g_skip = 0;
bool Skipped(Part p) { return (g_skip & p) != 0; }

// Telemetry, reported every ten seconds.
uint64_t g_frames = 0, g_panelFrames = 0, g_scanned = 0;
uint64_t g_turnedView = 0, g_turnedClip = 0, g_turnedOthers = 0, g_otherViews = 0, g_noPose = 0, g_offCentre = 0;
uint64_t g_anyFrame = 0, g_scaled = 0, g_masks = 0, g_pendingFull = 0, g_ambiguous = 0;
uint64_t g_lockDrawn = 0, g_lockDeclined = 0, g_lockWarped = 0, g_hudScaled = 0;
double g_lockWarpMaxDeg = 0;
float g_hudScale = 1.0f;  // experimental.onfoot_hud_scale
const char* g_lockWhy = "";
double g_lastYaw = 0, g_lastPitch = 0, g_lastRoll = 0;
ULONGLONG g_lastReport = 0;
bool g_loggedFirst = false, g_loggedLock = false;

bool Near(double v, double target, double tol) { return std::fabs(v - target) < tol; }

double RowLength(const float* r) {
    return std::sqrt(double(r[0]) * r[0] + double(r[1]) * r[1] + double(r[2]) * r[2]);
}

bool Unit(const float* v) { return std::fabs(RowLength(v) - 1) < 2e-3; }

void Release(ID3D11Buffer*& b) {
    if (b) b->Release();
    b = nullptr;
}

// rows: a clip transform's 4 rows. A reversed-Z perspective keeps the
// projection's row lengths and its (0, 0, 0, near) z row whatever the view;
// within 2% of the main one's is the main view.
bool IsMainView(const float* rows) {
    if (!g_haveMain) return false;
    constexpr double kTolerance = 0.02;
    const float* z = rows + 8;
    return std::fabs(z[0]) < 1e-4 && std::fabs(z[1]) < 1e-4 && std::fabs(z[2]) < 1e-4 &&
           Near(z[3], g_mainNear, kTolerance * g_mainNear) && Near(RowLength(rows + 12), 1, 1e-3) &&
           Near(RowLength(rows), g_mainScale[0], kTolerance * g_mainScale[0]) &&
           Near(RowLength(rows + 4), g_mainScale[1], kTolerance * g_mainScale[1]);
}

// The head's rotation in the game's view axes. headPose is OpenVR's: x right,
// y up, z back; the game's view is x right, y up, z forward (the clip
// transform's w row is the unit forward vector). Q = C R C, C = diag(1,1,-1).
void TakeHeadRotation() {
    if (g_qTaken) return;
    g_qTaken = true;
    float m[12];
    g_qValid = headPose(m);
    if (!g_qValid) return;
    const double s[3] = {1, 1, -1};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            g_q[i][j] = double(m[4 * i + j]) * s[i] * s[j];
            g_hRender[i][j] = m[4 * i + j];
        }
    // For the report: yaw about y, pitch about x, roll about z (degrees).
    const double kDeg = 57.29577951308232;
    g_lastYaw = std::atan2(g_q[0][2], g_q[2][2]) * kDeg;
    g_lastPitch = std::asin(std::fmax(-1.0, std::fmin(1.0, -g_q[1][2]))) * kDeg;
    g_lastRoll = std::atan2(g_q[1][0], g_q[1][1]) * kDeg;
}

bool Armed() {
    if (!g_panelLastFrame || !g_haveMain) return false;
    TakeHeadRotation();
    if (!g_qValid) {
        ++g_noPose;
        return false;
    }
    return true;
}

// rows: a clip transform (4 rows of 4), clip = P * V * M * x, with rows
// (sx v0, sy v1, z, sw v2): P a centred perspective, V the view, M any model
// transform with a uniform scale (sw; 1 without one). The turned view is
// Q^T V, so each view row becomes v'_i = sum_j Q[j][i] v_j. The z row is
// (0, 0, 0, near) for a reversed-Z infinite projection, or a * forward +
// (0, 0, 0, b) for a finite far plane, which turns with forward. Its inverse
// by rows (right/sx, up/sy, (0, 0, 0, 1/near), forward) has the same form.
// False (untouched) for an off-centre projection, which this does not model.
bool RotateRows(float* rows) {
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return false;
    const double dot = double(rows[0]) * rows[12] + double(rows[1]) * rows[13] + double(rows[2]) * rows[14];
    if (std::fabs(dot) > 1e-3 * sx * sw) {
        ++g_offCentre;
        return false;
    }
    double v[3][4];
    for (int k = 0; k < 4; ++k) {
        v[0][k] = rows[k] / sx;
        v[1][k] = rows[4 + k] / sy;
        v[2][k] = rows[12 + k] / sw;
    }
    const double a = double(rows[8]) * v[2][0] + double(rows[9]) * v[2][1] + double(rows[10]) * v[2][2];
    for (int k = 0; k < 3; ++k)
        if (std::fabs(rows[8 + k] - a * v[2][k]) > 1e-3 * (std::fabs(a) + 1)) return false;
    const double b = rows[11] - a * v[2][3];
    double n[3][4];
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 4; ++k) n[i][k] = g_q[0][i] * v[0][k] + g_q[1][i] * v[1][k] + g_q[2][i] * v[2][k];
    for (int k = 0; k < 4; ++k) {
        rows[k] = static_cast<float>(sx * n[0][k]);
        rows[4 + k] = static_cast<float>(sy * n[1][k]);
        rows[8 + k] = static_cast<float>(a * n[2][k] + (k == 3 ? b : 0.0));
        rows[12 + k] = static_cast<float>(sw * n[2][k]);
    }
    return true;
}

// A view of the main camera with a model scale folded into its projection:
// perspective, the main view's aspect (x/y scale ratio within 0.5%) and near
// plane (2%), any scale. On foot: the atmosphere sphere, whose b0 clip rows
// carry the planet radius (x 2954720, y 5252836 with the main 0.5625
// aspect); left to the 2% projection match, its horizon rode with the head.
// Left alone: the helmet HUD's wider projection (near 0.0675), which belongs
// to the head; orthographic shadow views (no w row); the square 256x256 view.
bool TurnSameCamera(float* rows) {
    if (!g_haveMain || Skipped(kPartOthers) || Skipped(kPartView)) return false;
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return false;
    const double aspect = g_mainScale[0] / g_mainScale[1];
    if (!Near(sx / sy, aspect, 5e-3 * aspect)) return false;
    // The z row stays (0, 0, 0, near) whatever the model scale.
    if (!Near(rows[11], g_mainNear, 0.02 * g_mainNear)) return false;
    if (!Armed() || !RotateRows(rows)) return false;
    ++g_turnedOthers;
    return true;
}

// The helmet HUD's view (experimental.onfoot_hud_scale): the main view's
// aspect, a narrower field (larger x scale) and its own near plane (0.0675,
// 0.1). Its x and y clip rows scaled by k draw the HUD k times its size about
// the centre -- with the head-locked view the panel's corners sit at the edge
// of the headset's field.
bool ScaleHud(float* rows) {
    if (g_hudScale == 1.0f || !g_haveMain || !g_panelLastFrame) return false;
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sy < 1e-6 || !Near(sw, 1, 1e-3) || sx < 1.1 * g_mainScale[0]) return false;
    const double aspect = g_mainScale[0] / g_mainScale[1];
    if (!Near(sx / sy, aspect, 5e-3 * aspect) || Near(rows[11], g_mainNear, 0.02 * g_mainNear)) return false;
    for (int i = 0; i < 8; ++i) rows[i] *= g_hudScale;
    ++g_hudScaled;
    return true;
}

// b0 at 64: the view's clip rows.
void TurnView(float* a) {
    if (!IsMainView(a)) {
        if (!TurnSameCamera(a) && !ScaleHud(a)) ++g_otherViews;
        return;
    }
    if (Skipped(kPartView) || !Armed() || !RotateRows(a)) return;
    ++g_turnedView;
    if (!g_loggedFirst) {
        g_loggedFirst = true;
        Log::get().note("onfoot look: first main-view write turned by the head (yaw %.1f, pitch %.1f, roll %.1f "
                        "degrees).",
                        g_lastYaw, g_lastPitch, g_lastRoll);
    }
}

// The frame's camera rotation from its (unturned) clip transform rows.
void TakeFrameRotation(const float* rows) {
    if (g_frameRValid) return;
    const double sx = RowLength(rows), sy = RowLength(rows + 4);
    if (sx < 1e-6 || sy < 1e-6) return;
    for (int k = 0; k < 3; ++k) {
        g_frameR[0][k] = rows[k] / sx;
        g_frameR[1][k] = rows[4 + k] / sy;
        g_frameR[2][k] = rows[12 + k];
    }
    g_frameRValid = true;
    double offIdentity = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            offIdentity = std::fmax(offIdentity, std::fabs(g_frameR[i][j] - (i == j ? 1.0 : 0.0)));
    g_frameRUsable = offIdentity > 1e-3;
    if (!g_frameRUsable) ++g_ambiguous;
}

// b1 at 4320: the same clip transform by columns. Learns the frame's R from
// the main view's; true when this write is the main view's.
bool TurnClip(float* c) {
    float rows[16];
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) rows[4 * r + col] = c[4 * col + r];
    if (!IsMainView(rows)) {
        if (TurnSameCamera(rows) || ScaleHud(rows))
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col) c[4 * col + r] = rows[4 * r + col];
        return false;
    }
    TakeFrameRotation(rows);
    if (Skipped(kPartView) || !Armed() || !RotateRows(rows)) return true;
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) c[4 * col + r] = rows[4 * r + col];
    ++g_turnedClip;
    return true;
}

// --- copies of the camera in other buffers ----------------------------------
//
// The deferred and sky passes carry their own copies of the camera
// (docs/cobra-onfoot-frame.md). Two forms are turned; each test starts
// with a few compares that reject almost every other 16 bytes, since every
// write of up to 16 KB is scanned at each float4.

bool Orthogonal(const float* a, const float* b, double la, double lb) {
    return std::fabs(double(a[0]) * b[0] + double(a[1]) * b[1] + double(a[2]) * b[2]) < 1e-3 * la * lb;
}

// "frames": the main camera through the main projection, or its inverse, in
// ANY world frame -- the star skybox's 592-byte cb2 carries the clip transform
// by columns (rows 7-10) and its inverse by rows (11-14) in the galaxy's
// frame, b1 rows 35-38 the inverse again, the deferred passes the planet
// frame's copy. The head turns the view axes whatever the world frame. Not a
// projection in VIEW space (identity axes: the shadow mask's inverse
// projection), which is already in the turned camera's axes.
bool MainCameraAnyFrame(const float* rows) {
    if (!g_haveMain) return false;
    const float* z = rows + 8;
    if (std::fabs(z[0]) > 1e-4 || std::fabs(z[1]) > 1e-4 || std::fabs(z[2]) > 1e-4) return false;
    const bool forward = Near(z[3], g_mainNear, 0.02 * g_mainNear);
    const bool inverse = Near(z[3], 1 / g_mainNear, 0.02 / g_mainNear);
    if (!forward && !inverse) return false;
    const double l0 = RowLength(rows), l1 = RowLength(rows + 4), l3 = RowLength(rows + 12);
    if (!Near(l3, 1, 1e-3)) return false;
    if (forward && !(Near(l0, g_mainScale[0], 0.02 * g_mainScale[0]) && Near(l1, g_mainScale[1], 0.02 * g_mainScale[1])))
        return false;
    if (inverse &&
        !(Near(l0, 1 / g_mainScale[0], 0.02 / g_mainScale[0]) && Near(l1, 1 / g_mainScale[1], 0.02 / g_mainScale[1])))
        return false;
    if (!Orthogonal(rows, rows + 4, l0, l1) || !Orthogonal(rows, rows + 12, l0, l3) ||
        !Orthogonal(rows + 4, rows + 12, l1, l3))
        return false;
    return !(std::fabs(rows[0] / l0 - 1) < 1e-3 && std::fabs(rows[5] / l1 - 1) < 1e-3 && std::fabs(rows[14] - 1) < 1e-3);
}

// "scaled": a clip transform of this frame's camera at any scale and
// projection, by rows (x along the camera's right axis, y along up, w along
// forward). The atmosphere sphere's 224-byte cb2 rows 0-3 are P V S, S the
// planet radius. Left alone, it shows as a thin fuzzy black bar that
// follows the head.
bool MatchesScaledClip(const float* rows) {
    const double sw = RowLength(rows + 12);
    if (sw < 1e-6) return false;
    for (int k = 0; k < 3; ++k)
        if (std::fabs(rows[12 + k] / sw - g_frameR[2][k]) > kMatch) return false;
    for (int r = 0; r < 2; ++r) {
        const double len = RowLength(rows + 4 * r);
        if (len < 0.1) return false;
        const double d =
            (rows[4 * r] * g_frameR[r][0] + rows[4 * r + 1] * g_frameR[r][1] + rows[4 * r + 2] * g_frameR[r][2]) / len;
        if (std::fabs(std::fabs(d) - 1) > kMatch) return false;
    }
    return true;
}

// Sixteen floats at f by rows or by columns, turned in place if test passes.
template <typename Test>
bool TurnMatrix(float* f, Test test) {
    for (const bool byColumns : {false, true}) {
        float m[16];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m[4 * r + c] = byColumns ? f[4 * c + r] : f[4 * r + c];
        if (!test(m) || !RotateRows(m)) continue;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) (byColumns ? f[4 * c + r] : f[4 * r + c]) = m[4 * r + c];
        return true;
    }
    return false;
}

// Every copy of the camera in a write gets the head rotation. skip: float
// ranges [from, to) left alone (b1's view slot, turned by the caller, and its
// previous pose).
struct Range {
    UINT from, to;
};
void TurnCopies(float* f, UINT floats, Range skipA = {}, Range skipB = {}) {
    // Without this frame's R only the frame-free test runs: the sky's block is
    // written before the frame's b1 names the camera.
    const bool haveR = g_frameRValid && g_frameRUsable;
    ++g_scanned;
    for (UINT o = 0; o + 16 <= floats; o += 4) {
        if ((o + 16 > skipA.from && o < skipA.to) || (o + 16 > skipB.from && o < skipB.to)) continue;
        float* b = f + o;
        // Cheap first: a projection's z row (by rows or by columns).
        const bool zRow = std::fabs(b[8]) < 1e-4 && std::fabs(b[9]) < 1e-4 && std::fabs(b[10]) < 1e-4;
        const bool zCol = std::fabs(b[2]) < 1e-4 && std::fabs(b[6]) < 1e-4 && std::fabs(b[10]) < 1e-4;
        if ((zRow || zCol) && !Skipped(kPartFrames) && TurnMatrix(b, MainCameraAnyFrame)) {
            ++g_anyFrame;
            o += 12;
            continue;
        }
        if (haveR && zRow && !Skipped(kPartScaled) && TurnMatrix(b, MatchesScaledClip)) {
            ++g_scaled;
            o += 12;
        }
    }
}

// "mask": the sun's screen-space shadow mask (full-screen PS
// 7EAC71963E66C5FE, an R8 target at panel size). Its 592-byte cb2: row 0 the
// target size, rows 1-4 the inverse projection, rows 7-9 the view-to-light
// rotation R L^T with w 0 -- each pixel's view position, rebuilt from depth,
// goes into light space through it. Unturned, every pixel swung by the head:
// pitch down, the ground left its shadows; pitch up, everything fell into
// them. The turned camera's is Q^T R L^T, Q^T times the rows.
bool TurnMask(float* f, UINT floats) {
    if (floats != 592 / 4 || Skipped(kPartMask)) return false;
    if (f[0] != static_cast<float>(g_panelW) || f[1] != static_cast<float>(g_panelH)) return false;
    float* v = f + 28;
    for (int i = 0; i < 3; ++i)
        if (!Unit(v + 4 * i) || v[4 * i + 3] != 0.0f) return false;
    double old[3][3];
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k) old[i][k] = v[4 * i + k];
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            v[4 * i + k] = static_cast<float>(g_q[0][i] * old[0][k] + g_q[1][i] * old[1][k] + g_q[2][i] * old[2][k]);
    ++g_masks;
    return true;
}

// Turns a mapped buffer. Map-discard memory is write-combined: reading it
// back is many times slower than ordinary memory (repeated passes over it
// cost 17 ms of CPU a frame on foot). So it is read ONCE into ordinary
// memory, matched and turned there, and written back only when turned.
void TurnBuffer(float* mapped, UINT floats) {
    static float work[kMaxScanBytes / 4];
    if (floats > kMaxScanBytes / 4) return;
    memcpy(work, mapped, floats * 4);
    const uint64_t before = g_anyFrame + g_scaled + g_masks;
    TurnCopies(work, floats);
    TurnMask(work, floats);
    if (before != g_anyFrame + g_scaled + g_masks) memcpy(mapped, work, floats * 4);
}

bool PanelGBufferBound() {
    const uint32_t rg = bindingGeneration(BindSlot::Rtv0), dg = bindingGeneration(BindSlot::Dsv0);
    if (rg == g_rtvGen && dg == g_dsvGen) return g_rtvIsPanelGBuffer;
    g_rtvGen = rg;
    g_dsvGen = dg;
    g_rtvIsPanelGBuffer = false;
    auto* rtv = static_cast<ID3D11RenderTargetView*>(bindingGet(BindSlot::Rtv0));
    if (!rtv || !bindingGet(BindSlot::Dsv0) || !g_panelW) return false;
    D3D11_RENDER_TARGET_VIEW_DESC rd;
    rtv->GetDesc(&rd);
    if (rd.Format != DXGI_FORMAT_R10G10B10A2_UNORM || rd.ViewDimension != D3D11_RTV_DIMENSION_TEXTURE2D) return false;
    ID3D11Resource* res = nullptr;
    rtv->GetResource(&res);
    if (!res) return false;
    ID3D11Texture2D* tex = nullptr;
    if (SUCCEEDED(res->QueryInterface(__uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&tex))) && tex) {
        D3D11_TEXTURE2D_DESC td;
        tex->GetDesc(&td);
        g_lastGbufW = td.Width;
        g_lastGbufH = td.Height;
        g_rtvIsPanelGBuffer = td.Width == g_panelW && td.Height == g_panelH;
        tex->Release();
    }
    res->Release();
    return g_rtvIsPanelGBuffer;
}

void Learn() {
    auto* view = static_cast<ID3D11Buffer*>(bindingGet(BindSlot::VsCb0));
    auto* frame = static_cast<ID3D11Buffer*>(bindingGet(BindSlot::VsCb1));
    if (!view || !frame) return;
    if (view != g_viewCb || frame != g_frameCb) {
        D3D11_BUFFER_DESC dv, df;
        view->GetDesc(&dv);
        frame->GetDesc(&df);
        if (dv.ByteWidth < kViewMinBytes || df.ByteWidth < kFrameMinBytes || df.ByteWidth > kMaxScanBytes) return;
        view->AddRef();
        frame->AddRef();
        Release(g_viewCb);
        Release(g_frameCb);
        g_viewCb = view;
        g_frameCb = frame;
        g_viewData = g_frameData = nullptr;
        g_frameFloats = df.ByteWidth / 4;
        g_viewBytes = dv.ByteWidth;
        g_viewShadowFresh = false;
        Log::get().note("onfoot look: panel scene buffers learned (b0 %u bytes, b1 %u bytes, panel %ux%u).",
                        dv.ByteWidth, df.ByteWidth, g_panelW, g_panelH);
        return;  // the shadow is of another buffer's writes; next frame
    }
    if (!g_viewShadowFresh) return;
    // The rows this draw uses are the last b0 write: the main view's. A turn
    // keeps the row lengths, so turned rows name the same projection.
    const float* z = g_viewShadow + 8;
    if (std::fabs(z[0]) > 1e-4 || std::fabs(z[1]) > 1e-4 || std::fabs(z[2]) > 1e-4 || z[3] <= 1e-6) return;
    const bool had = g_haveMain;
    g_mainScale[0] = RowLength(g_viewShadow);
    g_mainScale[1] = RowLength(g_viewShadow + 4);
    g_mainNear = z[3];
    g_haveMain = true;
    if (!had)
        Log::get().note("onfoot look: main view projection x %.4f y %.4f near %.4g.", g_mainScale[0], g_mainScale[1],
                        g_mainNear);
}

void Report() {
    const ULONGLONG now = GetTickCount64();
    if (!g_lastReport) g_lastReport = now;
    if (now - g_lastReport < 10000) return;
    g_lastReport = now;
    if (!g_panelFrames && !g_turnedView) {
        // On foot per the game and still nothing: say what WAS seen, so a
        // failed detection is not mistaken for "nothing to do".
        if (journalOnFootKnown() && journalOnFoot())
            Log::get().note("onfoot look: on foot, but no panel-sized G-buffer draw in the last 10 s (panel %ux%u; "
                            "last R10G10B10A2 target with depth %ux%u). Nothing turned.",
                            g_panelW, g_panelH, g_lastGbufW, g_lastGbufH);
        g_frames = 0;
        return;
    }
    using U = unsigned long long;
    Log::get().note("onfoot look: last 10 s %llu frames, %llu with the panel scene; turned %llu b0 and %llu b1 "
                    "main-view writes and %llu views of the main camera through another projection (%llu other views "
                    "left alone); copies turned: %llu in any frame, %llu scaled, %llu shadow masks, in "
                    "%llu scanned writes (%llu missed, every pending slot taken); %llu without a head pose, %llu "
                    "off-centre, %llu frames with the identity camera; head-locked view: %llu drawn, %llu declined%s%s%s, "
                    "%llu timewarped (largest %.2f degrees); %llu HUD views scaled; "
                    "head yaw %.1f pitch %.1f roll %.1f.",
                    U(g_frames), U(g_panelFrames), U(g_turnedView), U(g_turnedClip), U(g_turnedOthers),
                    U(g_otherViews), U(g_anyFrame), U(g_scaled), U(g_masks), U(g_scanned),
                    U(g_pendingFull), U(g_noPose), U(g_offCentre), U(g_ambiguous), U(g_lockDrawn), U(g_lockDeclined),
                    g_lockDeclined ? " (last: " : "", g_lockDeclined ? g_lockWhy : "", g_lockDeclined ? ")" : "",
                    U(g_lockWarped), g_lockWarpMaxDeg, U(g_hudScaled),
                    g_lastYaw, g_lastPitch,
                    g_lastRoll);
    g_frames = g_panelFrames = g_turnedView = g_turnedClip = g_turnedOthers = g_otherViews = 0;
    g_anyFrame = g_scaled = g_masks = g_scanned = g_pendingFull = 0;
    g_noPose = g_offCentre = g_ambiguous = 0;
    g_lockDrawn = g_lockDeclined = g_lockWarped = g_hudScaled = 0;
    g_lockWarpMaxDeg = 0;
}

// --- the head-locked view (onfoot_look.h) ----------------------------------

// b1's clip transform by columns as last written, after any turn: at the
// panel composite, the eye's (its projection times its view).
float g_eyeClip[16] = {};
bool g_eyeClipValid = false;

ID3D11Device* g_lockDevice = nullptr;
ID3D11VertexShader* g_lockVs = nullptr;
ID3D11Buffer* g_lockCb = nullptr;
ID3D11DepthStencilState* g_lockDss = nullptr;
bool g_lockFailed = false;
FaultBudget g_lockBudget("onfootLook.headLocked", 3);

// One quad from SV_VertexID, corners TL TR BL BR in clip space from b0,
// wound clockwise like the game's; the output signature is the game's
// composite VS's (vs_5C36AF051B98B9F1), which its pixel shader links to.
constexpr char kLockVs[] =
    "cbuffer C : register(b0) { float4 corner[4]; };\n"
    "struct O { float2 uv : __USER_VERTEX_M_TEXCOORD0; float4 pos : SV_Position; };\n"
    "static const uint k[6] = { 0, 1, 3, 0, 3, 2 };\n"
    "O main(uint id : SV_VertexID) {\n"
    "    const uint c = k[id];\n"
    "    O o;\n"
    "    o.uv = float2(c & 1, c >> 1);\n"
    "    o.pos = corner[c];\n"
    "    return o;\n"
    "}\n";

void ReleaseLock() {
    if (g_lockVs) g_lockVs->Release();
    if (g_lockCb) g_lockCb->Release();
    if (g_lockDss) g_lockDss->Release();
    g_lockVs = nullptr;
    g_lockCb = nullptr;
    g_lockDss = nullptr;
}

bool EnsureLock(ID3D11DeviceContext* ctx) {
    ID3D11Device* device = nullptr;
    ctx->GetDevice(&device);
    if (!device) return false;
    device->Release();  // the context holds it
    if (device == g_lockDevice && g_lockVs) return true;
    ReleaseLock();
    g_lockDevice = device;
    ID3DBlob* blob = nullptr;
    ID3DBlob* errors = nullptr;
    const HRESULT hr = D3DCompile(kLockVs, sizeof(kLockVs) - 1, "onfoot_lock_vs", nullptr, nullptr, "main", "vs_5_0",
                                  0, 0, &blob, &errors);
    if (errors) {
        Log::get().note("onfoot look: head-locked view shader: %s",
                        static_cast<const char*>(errors->GetBufferPointer()));
        errors->Release();
    }
    if (FAILED(hr) || !blob) return false;
    const bool vs =
        SUCCEEDED(device->CreateVertexShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &g_lockVs));
    blob->Release();
    D3D11_BUFFER_DESC bd{};
    bd.ByteWidth = 64;
    bd.Usage = D3D11_USAGE_DYNAMIC;
    bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    D3D11_DEPTH_STENCIL_DESC dd{};
    dd.DepthEnable = FALSE;
    dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
    dd.DepthFunc = D3D11_COMPARISON_ALWAYS;
    dd.StencilEnable = FALSE;
    if (!vs || FAILED(device->CreateBuffer(&bd, nullptr, &g_lockCb)) ||
        FAILED(device->CreateDepthStencilState(&dd, &g_lockDss))) {
        ReleaseLock();
        return false;
    }
    return true;
}

bool Decline(const char* why) {
    ++g_lockDeclined;
    g_lockWhy = why;
    return false;
}

double Dot3(const double* a, const double* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// The corners in the eye's clip space: directions F + X tx R + Y ty U, with
// (tx, ty) the main projection's tangents, projected by the eye's own rows.
// False when b1's slot does not hold an eye-like transform.
bool LockCorners(float out[16]) {
    const float* e = g_eyeClip;
    double row[4][3];
    for (int k = 0; k < 4; ++k)
        for (int i = 0; i < 3; ++i) row[k][i] = e[4 * i + k];
    const double lf = std::sqrt(Dot3(row[3], row[3]));
    if (!Near(lf, 1, 1e-2)) return false;
    double f[3], r[3], u[3];
    for (int i = 0; i < 3; ++i) f[i] = row[3][i] / lf;
    const double rf = Dot3(row[0], f), uf = Dot3(row[1], f);
    for (int i = 0; i < 3; ++i) {
        r[i] = row[0][i] - rf * f[i];
        u[i] = row[1][i] - uf * f[i];
    }
    const double lr = std::sqrt(Dot3(r, r)), lu = std::sqrt(Dot3(u, u));
    if (lr < 1e-3 || lu < 1e-3) return false;
    for (int i = 0; i < 3; ++i) {
        r[i] /= lr;
        u[i] /= lu;
    }
    // The timewarp: the scene was rendered for the head as it was when the
    // camera was turned (g_hRender); the eye is where the head is now. The
    // rotation between the two, D = H_now^T H_render (OpenVR axes: x right,
    // y up, z back), carries each corner from the rendered head's frame into
    // the current one, so the window is where the picture was taken.
    double dm[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float now[12];
    if (!Skipped(kPartTimewarp) && g_qValid && headPose(now)) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                dm[i][j] = now[i] * g_hRender[0][j] + now[4 + i] * g_hRender[1][j] + now[8 + i] * g_hRender[2][j];
        const double cosAngle = std::fmax(-1.0, std::fmin(1.0, (dm[0][0] + dm[1][1] + dm[2][2] - 1) / 2));
        const double deg = std::acos(cosAngle) * 57.29577951308232;
        if (deg > 0.01) ++g_lockWarped;
        g_lockWarpMaxDeg = std::fmax(g_lockWarpMaxDeg, deg);
    }
    const double tx = 1 / g_mainScale[0], ty = 1 / g_mainScale[1];
    const int kCorner[4][2] = {{-1, 1}, {1, 1}, {-1, -1}, {1, -1}};
    for (int c = 0; c < 4; ++c) {
        const double v[3] = {kCorner[c][0] * tx, kCorner[c][1] * ty, -1};
        double h[3];
        for (int i = 0; i < 3; ++i) h[i] = dm[i][0] * v[0] + dm[i][1] * v[1] + dm[i][2] * v[2];
        double d[3];
        for (int i = 0; i < 3; ++i) d[i] = h[0] * r[i] + h[1] * u[i] - h[2] * f[i];
        const double w = Dot3(row[3], d);
        if (w <= 0) return false;
        out[4 * c + 0] = static_cast<float>(Dot3(row[0], d));
        out[4 * c + 1] = static_cast<float>(Dot3(row[1], d));
        out[4 * c + 2] = static_cast<float>(0.5 * w);  // mid-depth; the depth test is off for the draw
        out[4 * c + 3] = static_cast<float>(w);
    }
    return true;
}

// The game's state the draw changes, at module scope so the fault path can
// put it back.
struct LockSaved {
    ID3D11VertexShader* vs = nullptr;
    ID3D11ClassInstance* inst[256] = {};
    UINT instCount = 256;
    ID3D11Buffer* cb0 = nullptr;
    ID3D11InputLayout* layout = nullptr;
    D3D11_PRIMITIVE_TOPOLOGY topo = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
    ID3D11DepthStencilState* dss = nullptr;
    UINT stencilRef = 0;
    bool held = false;
} g_lockSaved;

void LockRestore(ID3D11DeviceContext* ctx) {
    LockSaved& s = g_lockSaved;
    if (!s.held) return;
    s.held = false;
    ctx->VSSetShader(s.vs, s.instCount ? s.inst : nullptr, s.instCount);
    ctx->VSSetConstantBuffers(0, 1, &s.cb0);
    ctx->IASetInputLayout(s.layout);
    ctx->IASetPrimitiveTopology(s.topo);
    ctx->OMSetDepthStencilState(s.dss, s.stencilRef);
    if (s.vs) s.vs->Release();
    for (UINT i = 0; i < s.instCount; ++i)
        if (s.inst[i]) s.inst[i]->Release();
    if (s.cb0) s.cb0->Release();
    if (s.layout) s.layout->Release();
    if (s.dss) s.dss->Release();
    s = LockSaved{};
}

// --- capture: the census key (hotkey.dump_draws) on foot ---------------------
//
// A developer instrument. One whole on-foot frame of every write this module
// sees (b0, b1 and the pending buffers), AS THE GAME WROTE THEM (before any
// turn), to edvr_logs\onfoot_cb_<n>.bin in the flat mod's EDLSSCB1 layout:
// per write u32 index, i32 draw ordinal, u32 buffer id, u32 offset 0, u32
// size, char[16] method, u32 kept, bytes. The last record, buffer id
// 0xFFFFFFFF, is the frame's camera: R (rows right, up, forward) and the head
// Q, 18 floats. Captures taken facing different ways tell a world-fixed
// direction held in camera space from anything else
// (docs/cobra-onfoot-frame.md, Method).
FILE* g_capFile = nullptr;
uint32_t g_capNo = 0, g_capCount = 0, g_drawOrdinal = 0;
bool g_censusWasArmed = false;
const void* g_capPtrs[1024] = {};
uint32_t g_capPtrCount = 0;

uint32_t CapId(const void* p) {
    for (uint32_t i = 0; i < g_capPtrCount; ++i)
        if (g_capPtrs[i] == p) return i + 1;
    if (g_capPtrCount < 1024) g_capPtrs[g_capPtrCount++] = p;
    return g_capPtrCount;
}

void CapRecord(uint32_t id, const void* data, uint32_t bytes) {
    const char method[16] = "map-discard";
    const int32_t pass = static_cast<int32_t>(g_drawOrdinal);
    const uint32_t zero = 0;
    fwrite(&g_capCount, 4, 1, g_capFile);
    fwrite(&pass, 4, 1, g_capFile);
    fwrite(&id, 4, 1, g_capFile);
    fwrite(&zero, 4, 1, g_capFile);
    fwrite(&bytes, 4, 1, g_capFile);
    fwrite(method, 16, 1, g_capFile);
    fwrite(&bytes, 4, 1, g_capFile);
    fwrite(data, 1, bytes, g_capFile);
    ++g_capCount;
}

void CapWrite(ID3D11Resource* res, const void* data, UINT bytes) {
    if (g_capFile && data && bytes) CapRecord(CapId(res), data, bytes);
}

void CapBegin() {
    ++g_capNo;
    wchar_t name[64];
    swprintf_s(name, L"\\onfoot_cb_%u.bin", g_capNo);
    const std::wstring path = Config::get().logDir() + name;
    if (_wfopen_s(&g_capFile, path.c_str(), L"wb") != 0 || !g_capFile) {
        g_capFile = nullptr;
        Log::get().note("onfoot look: capture %u could not open its file.", g_capNo);
        return;
    }
    const uint32_t placeholder = 0;
    fwrite("EDLSSCB1", 8, 1, g_capFile);
    fwrite(&placeholder, 4, 1, g_capFile);
    g_capCount = 0;
    g_capPtrCount = 0;
}

void CapEnd() {
    float cam[18];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            cam[3 * i + j] = static_cast<float>(g_frameR[i][j]);
            cam[9 + 3 * i + j] = static_cast<float>(g_q[i][j]);
        }
    CapRecord(0xFFFFFFFFu, cam, sizeof(cam));
    fseek(g_capFile, 8, SEEK_SET);
    fwrite(&g_capCount, 4, 1, g_capFile);
    fclose(g_capFile);
    g_capFile = nullptr;
    Log::get().note("onfoot look: capture %u written (onfoot_cb_%u.bin): %u writes, %u buffers, camera %s, head "
                    "yaw %.1f pitch %.1f.",
                    g_capNo, g_capNo, g_capCount, g_capPtrCount, g_frameRValid ? "known" : "NOT known", g_lastYaw,
                    g_lastPitch);
}

}  // namespace

void onFootLookConfigure(Config& cfg) {
    const bool on = cfg.getBool("experimental.onfoot_head_look", false);
    if (on != detail::g_onFootLookEnabled)
        Log::get().note(on ? "onfoot look: ON. On foot, the head turns the game's own camera (the panel stays where "
                             "it is and shows the turned view)."
                           : "onfoot look: off.");
    detail::g_onFootLookEnabled = on;
    const bool locked = cfg.getBool("experimental.onfoot_head_locked", false);
    if (locked != detail::g_onFootHeadLocked)
        Log::get().note(locked ? "onfoot look: head-locked view ON (with the head look on, the on-foot screen is drawn "
                                 "in each eye at the game's own field of view)."
                               : "onfoot look: head-locked view off.");
    detail::g_onFootHeadLocked = locked;
    const float hud = cfg.getFloat("experimental.onfoot_hud_scale", 1.0f);
    const float hudClamped = hud < 0.3f ? 0.3f : (hud > 1.5f ? 1.5f : hud);
    if (hudClamped != g_hudScale) Log::get().note("onfoot look: helmet HUD drawn at %.2f of its size.", hudClamped);
    g_hudScale = hudClamped;
    const std::string skip = cfg.getString("experimental.onfoot_head_look_skip", "");
    unsigned mask = 0;
    const struct {
        const char* name;
        Part part;
    } kParts[] = {{"view", kPartView},     {"others", kPartOthers}, {"mask", kPartMask},
                  {"frames", kPartFrames}, {"scaled", kPartScaled}, {"timewarp", kPartTimewarp}};
    for (const auto& p : kParts)
        if (skip.find(p.name) != std::string::npos) mask |= p.part;
    if (mask != g_skip) Log::get().note("onfoot look: parts left out: \"%s\" (mask %u).", skip.c_str(), mask);
    g_skip = mask;
    if (!on) {
        g_viewData = g_frameData = nullptr;
        g_panelLastFrame = false;
    }
}

void onFootLookBeforeDraw() {
    if (!detail::g_onFootLookEnabled) return;
    ++g_drawOrdinal;
    if (g_foundThisFrame) return;
    if (!PanelGBufferBound()) return;
    g_foundThisFrame = true;
    Learn();
}

void onFootLookMapped(ID3D11Resource* res, void* data, D3D11_MAP type) {
    if (!detail::g_onFootLookEnabled || type == D3D11_MAP_READ || !res || res == g_lockCb) return;
    if (res == g_viewCb) {
        g_viewData = data;
        return;
    }
    if (res == g_frameCb) {
        g_frameData = data;
        return;
    }
    // Any other small constant or shader-resource buffer rewritten whole
    // while the panel scene runs: a candidate for camera copies.
    if (!g_panelLastFrame || type != D3D11_MAP_WRITE_DISCARD) return;
    D3D11_RESOURCE_DIMENSION dim;
    res->GetType(&dim);
    if (dim != D3D11_RESOURCE_DIMENSION_BUFFER) return;
    D3D11_BUFFER_DESC bd;
    static_cast<ID3D11Buffer*>(res)->GetDesc(&bd);
    if (bd.ByteWidth < 48 || bd.ByteWidth > kMaxScanBytes) return;
    if (!(bd.BindFlags & (D3D11_BIND_CONSTANT_BUFFER | D3D11_BIND_SHADER_RESOURCE))) return;
    for (Pending& p : g_pending) {
        if (!p.res || p.res == res) {
            p = {res, static_cast<float*>(data), bd.ByteWidth / 4};
            return;
        }
    }
    ++g_pendingFull;
}

void onFootLookBeforeUnmap(ID3D11Resource* res) {
    if (!detail::g_onFootLookEnabled || !res) return;
    if (g_capFile) {
        if (res == g_viewCb && g_viewData) CapWrite(res, g_viewData, g_viewBytes);
        else if (res == g_frameCb && g_frameData) CapWrite(res, g_frameData, g_frameFloats * 4);
        else
            for (const Pending& p : g_pending)
                if (p.res == res) CapWrite(res, p.data, p.floats * 4);
    }
    if (res == g_viewCb && g_viewData) {
        float* a = reinterpret_cast<float*>(static_cast<char*>(g_viewData) + kViewOffset);
        TurnView(a);
        memcpy(g_viewShadow, a, sizeof(g_viewShadow));
        g_viewShadowFresh = true;
        g_viewData = nullptr;
    } else if (res == g_frameCb && g_frameData) {
        // The view slot (4320) is turned only for the main view; every b1
        // write also carries the camera's own copies (R^T at 3664, R at 4432,
        // the sky's inverse at 560), whichever view the slot holds. Worked on
        // in ordinary memory and written back whole (see TurnBuffer).
        static float work[kMaxScanBytes / 4];
        memcpy(work, g_frameData, g_frameFloats * 4);
        TurnClip(work + kClipOffset / 4);
        if (Armed())
            TurnCopies(work, g_frameFloats, {kClipOffset / 4, kClipOffset / 4 + 16},
                       {kPrevPoseOffset / 4, kPrevPoseOffset / 4 + 12});
        memcpy(g_frameData, work, g_frameFloats * 4);
        memcpy(g_eyeClip, work + kClipOffset / 4, sizeof(g_eyeClip));
        g_eyeClipValid = true;
        g_frameData = nullptr;
    } else {
        for (Pending& p : g_pending) {
            if (p.res != res) continue;
            if (Armed()) TurnBuffer(p.data, p.floats);
            p = {};
            break;
        }
    }
}

void onFootLookStateCleared() {
    g_rtvGen = g_dsvGen = 0;
    g_rtvIsPanelGBuffer = false;
}

void onFootLookFrameBoundary() {
    if (!detail::g_onFootLookEnabled) return;
    ++g_frames;
    if (g_foundThisFrame) ++g_panelFrames;
    g_panelLastFrame = g_foundThisFrame;
    g_foundThisFrame = false;
    // The capture: the frame that just ended, then (on the census key's rising
    // edge, on foot) the next one.
    if (g_capFile) CapEnd();
    const bool armed = drawCensusArmed();
    if (armed && !g_censusWasArmed && g_panelLastFrame) CapBegin();
    g_censusWasArmed = armed;
    g_drawOrdinal = 0;
    g_qTaken = false;
    g_frameRValid = false;
    for (Pending& p : g_pending) p = {};
    uint32_t w = 0, h = 0;
    if (vScreenPanelSize(&w, &h) && (w != g_panelW || h != g_panelH)) {
        g_panelW = w;
        g_panelH = h;
        g_rtvGen = g_dsvGen = 0;  // re-test the bound target against the new size
    }
    Report();
}

bool onFootLookDrawHeadLocked(ID3D11DeviceContext* ctx, OnFootDrawFn draw) {
    if (!onFootLookHeadLockedWanted() || g_lockFailed || !ctx || !draw) return false;
    if (!g_haveMain || !(g_foundThisFrame || g_panelLastFrame)) return Decline("no panel scene");
    if (!g_eyeClipValid || bindingGet(BindSlot::VsCb1) != g_frameCb) return Decline("the composite reads another b1");
    float corners[16];
    if (!LockCorners(corners)) return Decline("b1's slot is not an eye transform");
    bool drawn = false;
    const bool ok = guardedBudget(g_lockBudget, [&] {
        if (!EnsureLock(ctx)) {
            g_lockFailed = true;
            Log::get().note("onfoot look: head-locked view could not build its shader or states; the screen stays.");
            return;
        }
        D3D11_MAPPED_SUBRESOURCE m{};
        if (FAILED(ctx->Map(g_lockCb, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)) || !m.pData) return;
        memcpy(m.pData, corners, sizeof(corners));
        ctx->Unmap(g_lockCb, 0);
        LockSaved& s = g_lockSaved;
        ctx->VSGetShader(&s.vs, s.inst, &s.instCount);
        ctx->VSGetConstantBuffers(0, 1, &s.cb0);
        ctx->IAGetInputLayout(&s.layout);
        ctx->IAGetPrimitiveTopology(&s.topo);
        ctx->OMGetDepthStencilState(&s.dss, &s.stencilRef);
        s.held = true;
        ctx->VSSetShader(g_lockVs, nullptr, 0);
        ctx->VSSetConstantBuffers(0, 1, &g_lockCb);
        ctx->IASetInputLayout(nullptr);
        ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        ctx->OMSetDepthStencilState(g_lockDss, 0);
        draw(ctx, 6, 0);
        LockRestore(ctx);
        drawn = true;
    });
    if (!ok) {
        guarded("onfootLook.headLockedRestore", [&] { LockRestore(ctx); });
        g_lockFailed = true;
        Log::get().note("onfoot look: head-locked view faulted; the screen stays for this session.");
        return false;
    }
    if (!drawn) return Decline("the draw could not be set up");
    if (++g_lockDrawn == 1 && !g_loggedLock) {
        g_loggedLock = true;
        Log::get().note("onfoot look: head-locked view drawn (tangents x %.3f y %.3f).", 1 / g_mainScale[0],
                        1 / g_mainScale[1]);
    }
    return true;
}

}  // namespace edvr
