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
#include "camera_hunt.h"
#include "draw_census.h"
#include "head_drive.h"
#include "journal_watch.h"
#include "mem_probe.h"
#include "stereo_mode_probe.h"
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
// b1: the camera's rotation by rows (right, up, forward; float4 each), as the
// game wrote it -- what camera_hunt.h looks for in the game's memory.
constexpr UINT kCamRowsOffset = 4432;
float g_rawCam[12] = {};
bool g_rawCamValid = false;

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
bool g_mainCentred = false;  // the x row has no forward part: a flat view, not an eye's asymmetric one

// THE MAIN PROJECTION IS THE WIDEST. It was the first G-buffer draw's, until
// the aim-down-sights capture of 2026-09-24: aiming, the frame's first
// G-buffer draw is the gun, through its own narrower projection (x 2.10
// against the world's 1.15), and the world became "another camera" -- the
// sky's copies unturned (the black bar back), the placed window the gun's
// size. Every G-buffer draw's b0, as the game wrote it, is weighed instead;
// the widest is the world's, and it holds from the next frame.
float g_viewRaw[16] = {};
bool g_viewRawFresh = false;
bool g_haveCand = false, g_candCentred = false;
double g_candScale[2] = {}, g_candNear = 0;
uint32_t g_mainLogs = 0;

// Per-frame learning, and whether the panel scene ran last frame (the turn
// is armed only then: in the cockpit the G-buffer is eye-sized).
bool g_foundThisFrame = false;
bool g_panelLastFrame = false;
uint32_t g_panelW = 0, g_panelH = 0;  // refreshed each frame

// A target of the on-foot scene: the panel's shape at any render scale. The
// game's supersampling and upscaling render the scene smaller or larger than
// the panel (EDVR's vscreen_res_width); taken at exactly the panel's size,
// the "VR Medium" preset's lower scale turned the head look, the eyes and
// the shadow mask off (2026-09-24).
bool SceneSized(double w, double h) {
    if (!g_panelW || !g_panelH || w < 1 || h < 1) return false;
    if (std::fabs(w * g_panelH - h * g_panelW) > 0.01 * w * g_panelH) return false;
    return w >= 0.3 * g_panelW && w <= 2.5 * g_panelW;
}

// The render-target check, cached by the binding shadow's generations.
uint32_t g_rtvGen = 0, g_dsvGen = 0;
bool g_stereoDiag = false;  // experimental.onfoot_stereo_diag: eye-sized G-buffers count too
bool g_rtvIsPanelGBuffer = false;
bool g_rtvIsAnyGBuffer = false;  // the diagnostic's: any R10G10B10A2 target with depth, 1024 wide or more
// The last R10G10B10A2 target with a depth bound, whatever its size: what the
// "on foot but no panel scene" line reports, so a size mismatch is visible.
uint32_t g_lastGbufW = 0, g_lastGbufH = 0;

// The rotation this frame's writes are turned by, in the game's view axes:
// the head's, taken once per frame at the first turn -- or, with the head
// driving the game's look (head_drive.h), what is left of it once the game's
// camera has turned: Q = G^T H, G the drawn camera as the head sample the
// game applied (known at the frame's first main view; the frame's earlier
// writes get the last frame's G).
double g_q[3][3] = {};
double g_qHead[3][3] = {};
double g_qGame[3][3] = {};
bool g_haveQGame = false, g_driveTaken = false;
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
// Floats of a buffer moved with the eye (the stereo below; InverseEdits):
// each by coef per metre of the eye's offset.
struct Edit {
    UINT index;  // the float in the buffer
    float coef;
};
constexpr int kMaxEdits = 32;
struct EditList {
    Edit e[kMaxEdits];
    int n;
};
// How close a copy has to be: the lighting passes compute their copies of the
// camera apart from the view slot's (up to 2e-4 off).
constexpr double kMatch = 1e-3;

// Developer switches (experimental.onfoot_head_look_skip): parts of the turn
// left out, to see by eye which one a visual fault follows. Live.
enum Part : unsigned {
    kPartView = 1,
    kPartOthers = 2,
    kPartMask = 4,
    kPartFrames = 8,
    kPartScaled = 16,
    kPartTimewarp = 32,
    kPartHudEye = 64,  // the on-foot stereo's move of the helmet HUD's eye
    kPartEyeLight = 128,  // the on-foot stereo's move of the camera's inverses (the lighting's eye)
};
unsigned g_skip = 0;
bool Skipped(Part p) { return (g_skip & p) != 0; }

// Telemetry, reported every ten seconds.
uint64_t g_frames = 0, g_panelFrames = 0, g_scanned = 0;
uint64_t g_turnedView = 0, g_turnedClip = 0, g_turnedOthers = 0, g_otherViews = 0, g_noPose = 0, g_offCentre = 0;
uint64_t g_anyFrame = 0, g_scaled = 0, g_masks = 0, g_pendingFull = 0, g_ambiguous = 0;
uint64_t g_lockDrawn = 0, g_lockDeclined = 0, g_lockWarped = 0, g_hudScaled = 0, g_modelMatched = 0, g_hudTurned = 0;
double g_lockWarpMaxDeg = 0;
float g_hudScale = 1.0f;  // experimental.onfoot_hud_scale
bool g_matchFov = true;   // experimental.onfoot_match_fov
const char* g_lockWhy = "";
double g_lastYaw = 0, g_lastPitch = 0, g_lastRoll = 0;
ULONGLONG g_lastReport = 0;
bool g_loggedFirst = false, g_loggedLock = false;

// --- the on-foot stereo's eyes (experimental.onfoot_stereo) ------------------
//
// With the engine kept in HMD stereo on foot (stereo_mode_probe.h), both eye
// pipelines render the whole flat scene, each into its own targets, the two
// interleaved stage by stage: the census of 2026-09-24 saw the depth, the
// G-buffer and the lighting passes alternate between two panel-sized depth
// targets, and each pipeline's final is the texture the game submits for one
// eye. Both read the same camera -- the body camera, camera-relative (the
// eye at the origin), no head, no eye offset -- through the shared b0 and b1,
// rewritten for every pass. The head look turns it for both; this moves each
// pipeline's eye half the IPD along the turned camera's right axis, and tells
// the runtime to place each image at the flat frustum's own angles inside
// the eye's field (frame_flag.h, onFootFlat), where the headset's compositor
// reprojects it like any eye image.
//
// Moving the eye is one number per clip transform. Camera-relative, clip =
// P V (x - e) for an eye at e, and e along the camera's right axis moves only
// the x row's translation, by -sx * offset, sx the projection's x scale (the
// x row's length over the w row's, whatever model scale the transform
// carries). The writes moved are the main camera's: the main view, the same
// camera through a model, and the helmet HUD (part "hudeye"), in b0's rows at
// 64 and b1's slot at 4320. The camera's inverses -- the lighting's -- are
// moved to the same eye (InverseEdits), in b1 and in any other buffer the
// head look turns them in, and those buffers are written again for the other
// eye like b0 and b1. A lighting pass binds no depth target of theirs: its
// eye is the pipeline whose depth it reads as a texture, else whose target
// it draws into.
//
// WHICH EYE. A pass's first camera write comes while the other pipeline's
// targets are still bound (the write before a pipeline's first draw follows
// the other's last one). So each write is moved for the pipeline that last
// drew, the unmoved write is kept, and at a draw into the other pipeline's
// depth target the buffer is written again for it -- a Map-discard through
// the real Map (onFootLookSetMapFns), which this module's tee never sees. The
// pipelines are told apart by their panel-sized depth targets: a new one
// belongs to the pipeline other than the last one that drew (their depth,
// then the HUD's pair after both lightings). Which pipeline is the LEFT eye:
// the texture the game submits for the left eye (frame_flag.h,
// gameSubmitted) is one of that pipeline's render targets.
OnFootMapFn g_realMap = nullptr;
OnFootUnmapFn g_realUnmap = nullptr;
float g_ipdMm = 0;  // experimental.onfoot_stereo_ipd_mm: 0 the headset's, below 0 no move
bool g_stereoOn = false, g_stereoWasOn = false;  // this frame: on-foot stereo, the image flat
double g_halfIpd = 0;                            // metres, this frame
int g_leftPipe = 0;                              // the pipeline (0 = the first seen) that is the left eye
bool g_leftKnown = false, g_leftGuessNoted = false;
int g_curPipe = -1, g_lastPipe = -1;  // the draw's pipeline, the last known one
uint32_t g_pipeDsvGen = 0, g_pipeRtvGen = 0;
struct PipeTarget {
    void* res;
    int pipe;
};
constexpr int kMaxPipeTargets = 8;
PipeTarget g_pipeTargets[kMaxPipeTargets] = {};
int g_pipeTargetCount = 0;
void* g_pipeRtvs[2][8] = {};
int g_pipeRtvCount[2] = {};
uint64_t g_stereoFrames = 0;
// The last camera writes as turned and unmoved, how much to move them per
// metre, and for which pipeline they were written. b0 is 208 bytes.
constexpr UINT kB0Moved = kViewOffset / 4 + 3;   // row 0, column 3
constexpr UINT kB1Moved = kClipOffset / 4 + 12;  // by columns: column 3, row 0
float g_b0Copy[64] = {};
float g_b1Copy[kMaxScanBytes / 4] = {};
UINT g_b0CopyBytes = 0, g_b1CopyBytes = 0;
EditList g_b0Edits{}, g_b1Edits{};  // the last writes' moves (none: not moved)
// experimental.onfoot_stereo_near_eye: the share of the eye offset given to
// writes in VIEW space (the camera's own axes, before the head's turn: the
// first-person body and what it holds). A first-person weapon drawn smaller
// and nearer than it is -- so it does not pass through walls -- takes the
// whole offset as if it were that near, and the eyes cannot fuse it (the
// pistol, flight of 2026-09-24).
double g_nearEye = 1.0;
int g_b0Pipe = -1, g_b1Pipe = -1;
bool g_viewDiscard = false, g_frameDiscard = false;  // the game's Map of b0 / b1 was a discard
uint64_t g_moved = 0, g_movedNear = 0, g_rewrites = 0, g_rewriteFails = 0, g_notDiscard = 0, g_newTargets = 0;
// Other buffers holding the camera's inverses, kept as turned and unmoved
// (a reference held), written again when a draw of the other pipeline comes.
struct Tracked {
    ID3D11Buffer* buf;
    UINT bytes;
    int pipe;
    EditList edits;
    float copy[kMaxScanBytes / 4];
};
constexpr int kMaxTracked = 8;
Tracked g_tracked[kMaxTracked];
int g_trackedCount = 0;
uint64_t g_invMoved = 0, g_trackedFull = 0, g_trackedRewrites = 0, g_pipeFromSrv = 0, g_pipeFromRtv = 0;
int g_drawPipe = -1, g_drawPipeSrc = 0;  // the last draw's, for the capture
enum PipeSource { kFromNone = 0, kFromDsv = 1, kFromSrv = 2, kFromRtv = 3 };
int g_fbPipe = -1, g_fbSrc = kFromNone;
uint32_t g_fbGen[5] = {};


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

// g_q from the head's rotation and, when the head drives the game, the
// game's camera: Q = G^T H.
void ComposeQ() {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            g_q[i][j] = !g_haveQGame ? g_qHead[i][j]
                                     : g_qGame[0][i] * g_qHead[0][j] + g_qGame[1][i] * g_qHead[1][j] +
                                           g_qGame[2][i] * g_qHead[2][j];
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
            g_qHead[i][j] = double(m[4 * i + j]) * s[i] * s[j];
            g_hRender[i][j] = m[4 * i + j];
        }
    // For the report: yaw about y, pitch about x, roll about z (degrees).
    const double kDeg = 57.29577951308232;
    g_lastYaw = std::atan2(g_qHead[0][2], g_qHead[2][2]) * kDeg;
    g_lastPitch = std::asin(std::fmax(-1.0, std::fmin(1.0, -g_qHead[1][2]))) * kDeg;
    g_lastRoll = std::atan2(g_qHead[1][0], g_qHead[1][1]) * kDeg;
    ComposeQ();
}

// The frame's first main view, before it turns: which head sample the game
// built this camera from, when the head drives it.
void DriveFromRows(const float* rows) {
    if (g_driveTaken) return;
    g_driveTaken = true;
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return;
    double axes[3][3];
    for (int k = 0; k < 3; ++k) {
        axes[0][k] = rows[k] / sx;
        axes[1][k] = rows[4 + k] / sy;
        axes[2][k] = rows[12 + k] / sw;
    }
    g_haveQGame = headDriveCamera(axes, g_qGame);
    if (!g_haveQGame && headDriveActive()) {
        // The camera has the head in it, which sample unknown: no residual.
        memcpy(g_qGame, g_qHead, sizeof(g_qGame));
        g_haveQGame = true;
    }
    ComposeQ();
    if (g_haveQGame) {
        const double c = std::fmax(-1.0, std::fmin(1.0, (g_q[0][0] + g_q[1][1] + g_q[2][2] - 1) / 2));
        headDriveNoteResidual(std::acos(c) * 57.29577951308232);
    }
}

// Only on foot, with a centred main view. With the on-foot stereo the ship,
// once visited on foot, keeps rendering into panel-sized targets (the
// display mode never changes back), so the panel test alone passes in the
// cockpit: its asymmetric eye views were refused as off-centre, but the
// shadow mask turned, and the cockpit's shadows swung with the head (flight
// of 2026-09-24). With the stereo holding, the engine says where the player
// is (stereo_mode_probe.h); not the journal, which after a load on foot
// still holds the last session's Embark.
bool Armed() {
    if (!g_panelLastFrame || !g_haveMain || !g_mainCentred) return false;
    if (onFootStereoHolding() && !onFootStereoWanted()) return false;
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
    // The z row stays (0, 0, 0, near) whatever the model scale; so does the
    // field (the x scale over the model scale). A narrower field is the
    // view model's (MatchFov), which stays with the head as it always has.
    if (!Near(rows[11], g_mainNear, 0.02 * g_mainNear)) return false;
    if (!Near(sx / sw, g_mainScale[0], 0.02 * g_mainScale[0])) return false;
    if (!Armed() || !RotateRows(rows)) return false;
    ++g_turnedOthers;
    return true;
}

// A perspective's z row: (0, 0, 0, near) for the world's infinite far
// plane, or a * forward + (0, 0, 0, b) for a finite one -- the HUD pass's
// (near 0.27 or 0.1, far about 1000). False for anything else.
bool PerspectiveZ(const float* rows, bool* finite) {
    const float* z = rows + 8;
    if (z[3] <= 0) return false;
    const double zl = RowLength(z);
    *finite = zl > 1e-7;
    if (!*finite) return true;
    const double wl = RowLength(rows + 12);
    if (wl < 1e-6) return false;
    const double a = (double(z[0]) * rows[12] + double(z[1]) * rows[13] + double(z[2]) * rows[14]) / (wl * wl);
    for (int k = 0; k < 3; ++k)
        if (std::fabs(z[k] - a * rows[12 + k]) > 1e-3 * zl) return false;
    return true;
}

// A clip transform's rows (by rows) whose axes are the identity's: a write
// in view space, taken before the head's turn.
bool ViewSpaceRows(const float* rows) {
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return false;
    return std::fabs(rows[0] / sx - 1) < 1e-4 && std::fabs(rows[5] / sy - 1) < 1e-4 && std::fabs(rows[14] / sw - 1) < 1e-4;
}

// THE VIEW MODEL AND THE HUD (the censuses of 2026-09-24), two kinds of view
// of the camera the world is not drawn through, rows as the game wrote them.
//
// The view model -- the first-person body and what it holds, drawn into the
// G-buffer in the world's axes -- through a narrower projection than the
// world's: x 1.294 against 0.5625 at the hip and the widest FOV setting,
// 2.10 against 1.15 aiming, so on a flat screen it looks bigger and nearer
// than it is. In the headset the world is 1:1, so with
// experimental.onfoot_match_fov the view model is drawn through the world's
// projection: x and y rows scaled by the world's x scale over its own.
//
// The HUD pass -- health, the compass, the ammo readout -- in view space
// with a finite far plane, drawn after the scene into each eye's final. A
// flat overlay laid out for the screen, whose corners at the widest field
// sit at the edge of sight: experimental.onfoot_hud_scale scales it about
// the centre (0.5 half). The first build to match the field scaled both by
// hud_scale and missed the HUD (its finite far plane): the hands stretched,
// the HUD stayed at the edges.
bool MatchFov(float* rows) {
    if (!g_haveMain || !g_panelLastFrame || !g_mainCentred) return false;
    if (onFootStereoHolding() && !onFootStereoWanted()) return false;
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return false;
    const double aspect = g_mainScale[0] / g_mainScale[1];
    if (!Near(sx / sy, aspect, 5e-3 * aspect)) return false;
    bool finite = false;
    if (!PerspectiveZ(rows, &finite)) return false;
    const double dot = double(rows[0]) * rows[12] + double(rows[1]) * rows[13] + double(rows[2]) * rows[14];
    if (std::fabs(dot) > 1e-3 * sx * sw) return false;
    double k;
    if (finite && ViewSpaceRows(rows)) {
        if (g_hudScale == 1.0f) return false;
        k = g_hudScale;
        ++g_hudScaled;
    } else {
        const double sxn = sx / sw;
        if (!g_matchFov || sxn < 1.02 * g_mainScale[0]) return false;
        k = g_mainScale[0] / sxn;
        ++g_modelMatched;
    }
    for (int i = 0; i < 8; ++i) rows[i] = static_cast<float>(rows[i] * k);
    return true;
}

// THE HUD WITH THE HEAD DRIVING THE GAME. The picture turns by what is left
// of the head once the game's camera has turned (G^T H); the view model --
// the gun -- is drawn along the game's camera and is not turned, and the
// shots go along it too. The HUD, drawn in view space, would stay with the
// head, and its crosshair a degree or two off the shots whenever the game's
// camera is not quite where the head is: the frames it trails the head, the
// stick turn's lead, recoil, aim assist (flight of 2026-09-24 18:47: "a hair
// off center"). So the HUD pass's views (view space, finite far plane) turn
// with the rest: the HUD stays on the game's camera, the crosshair on the
// shots.
bool TurnHud(float* rows) {
    if (!g_haveQGame || !ViewSpaceRows(rows)) return false;
    bool finite = false;
    if (!PerspectiveZ(rows, &finite) || !finite) return false;
    if (!Armed() || !RotateRows(rows)) return false;
    ++g_hudTurned;
    return true;
}

// b0 at 64: the view's clip rows.
void TurnView(float* a) {
    if (!IsMainView(a)) {
        const bool turned = TurnSameCamera(a);
        const bool matched = MatchFov(a);
        const bool hud = TurnHud(a);
        if (!turned && !matched && !hud) ++g_otherViews;
        return;
    }
    if (Skipped(kPartView) || !Armed()) return;
    DriveFromRows(a);
    if (!RotateRows(a)) return;
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
        const bool turned = TurnSameCamera(rows);
        const bool matched = MatchFov(rows);
        const bool hud = TurnHud(rows);
        if (turned || matched || hud)
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col) c[4 * col + r] = rows[4 * r + col];
        return false;
    }
    TakeFrameRotation(rows);
    if (Skipped(kPartView) || !Armed()) return true;
    DriveFromRows(rows);
    if (!RotateRows(rows)) return true;
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
bool TurnMatrix(float* f, Test test, bool* byColumnsOut = nullptr) {
    for (const bool byColumns : {false, true}) {
        float m[16];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m[4 * r + c] = byColumns ? f[4 * c + r] : f[4 * r + c];
        if (!test(m) || !RotateRows(m)) continue;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) (byColumns ? f[4 * c + r] : f[4 * r + c]) = m[4 * r + c];
        if (byColumnsOut) *byColumnsOut = byColumns;
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

// THE LIGHTING'S EYE (part "eyelight"). The lighting rebuilds each pixel's
// position from its depth through an inverse of the camera: the "frames"
// copies, and the shadow mask's inverse projection in view space. By rows
// (after any transpose) such an inverse is right/sx, up/sy, (0,0,0,1/near),
// forward: the position comes out as clip x right/sx + ... with w = clip
// z/near. The pixel was drawn from the moved eye, so it comes out relative
// to that eye, and is lit against the centre's lights and shadows: off by
// the eye's offset, each eye the other way (the ~3 cm of the first stereo
// flight; gone with the eyes not moved, 2026-09-24). Rebuilt for the eye
// e = offset * right, the position gains e * clip z / near: the z row's xyz
// gains e / near. At infinity (clip z 0, the reversed depth) nothing
// changes: a ray to the sky stays as it was.

// Sixteen floats at b (buffer float index at): an inverse of the main
// projection, in any frame or in view space, by rows or by columns: its z
// row's edits appended.
bool InverseEdits(const float* b, UINT at, EditList* out) {
    if (!out || !g_haveMain || out->n + 3 > kMaxEdits) return false;
    for (const bool byColumns : {false, true}) {
        float m[16];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m[4 * r + c] = byColumns ? b[4 * c + r] : b[4 * r + c];
        if (std::fabs(m[8]) > 1e-4 || std::fabs(m[9]) > 1e-4 || std::fabs(m[10]) > 1e-4) continue;
        if (!Near(m[11], 1 / g_mainNear, 0.02 / g_mainNear)) continue;
        const double l0 = RowLength(m);
        if (!Near(l0, 1 / g_mainScale[0], 0.02 / g_mainScale[0]) || !Near(RowLength(m + 12), 1, 1e-3)) continue;
        for (int k = 0; k < 3; ++k)
            out->e[out->n++] = {at + (byColumns ? 4 * k + 2 : 8 + k), static_cast<float>(m[k] / l0 * m[11])};
        return true;
    }
    return false;
}

// Sixteen floats (unturned): a copy of the camera in the frame's own world
// frame -- its right and forward rows (either layout) along this frame's.
uint64_t g_invOtherFrame = 0;
bool InMainFrame(const float* b) {
    for (const bool byColumns : {false, true}) {
        double r[3], f[3];
        for (int k = 0; k < 3; ++k) {
            r[k] = byColumns ? b[4 * k] : b[k];
            f[k] = byColumns ? b[4 * k + 3] : b[12 + k];
        }
        const double rl = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        const double fl = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        if (rl < 1e-6 || fl < 1e-6) continue;
        double dr = 0, df = 0;
        for (int k = 0; k < 3; ++k) {
            dr += r[k] / rl * g_frameR[0][k];
            df += f[k] / fl * g_frameR[2][k];
        }
        if (dr > 1 - kMatch && df > 1 - kMatch) return true;
    }
    return false;
}

// THE LIGHTING'S CAMERA (with "eyelight"; buffers other than b0 and b1).
// The deferred lights' constants (a 784-byte cb2, capture of 2026-09-25)
// hold the camera in the frame's world with its position: the view V = R|t
// (rows 2-5), its inverse R^T|c (rows 6-9) and P V (rows 10-13). Written
// once for both eyes, the moved eye was lit from the other's place. For the
// eye e = offset * right: V's translation loses offset in its right row, the
// inverse's position gains e, P V's x row loses sx * offset. (R the game's
// camera, not turned by the head: the head's residual turn of the eye's axis
// is a millimetre at most.)
uint64_t g_basesMoved = 0, g_centreDraws = 0, g_skippedDraws = 0;
bool BasisEdits(const float* b, UINT at, EditList* out) {
    if (!out || out->n + 3 > kMaxEdits) return false;
    for (const bool byColumns : {false, true}) {
        float m[16];
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) m[4 * r + c] = byColumns ? b[4 * c + r] : b[4 * r + c];
        if (m[12] != 0.0f || m[13] != 0.0f || m[14] != 0.0f || m[15] != 1.0f) continue;
        bool view = true, inverse = true;
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k) {
                view &= std::fabs(m[4 * i + k] - g_frameR[i][k]) < 1e-3;
                inverse &= std::fabs(m[4 * i + k] - g_frameR[k][i]) < 1e-3;
            }
        auto index = [&](int r, int c) { return at + (byColumns ? 4 * c + r : 4 * r + c); };
        if (view) {
            out->e[out->n++] = {index(0, 3), -1.0f};
            return true;
        }
        if (inverse) {
            for (int k = 0; k < 3; ++k) out->e[out->n++] = {index(k, 3), static_cast<float>(g_frameR[0][k])};
            return true;
        }
    }
    return false;
}

// A forward copy of the main camera in the frame's world (turned; either
// layout): its x row's translation, as b0's.
bool ForwardEdits(const float* b, UINT at, bool byColumns, EditList* out) {
    if (!out || out->n + 1 > kMaxEdits) return false;
    float m[16];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) m[4 * r + c] = byColumns ? b[4 * c + r] : b[4 * r + c];
    if (!Near(m[11], g_mainNear, 0.02 * g_mainNear)) return false;
    const double sw = RowLength(m + 12);
    if (sw < 1e-6) return false;
    out->e[out->n++] = {at + (byColumns ? 12 : 3), static_cast<float>(-RowLength(m) / sw)};
    return true;
}

void TurnCopies(float* f, UINT floats, Range skipA = {}, Range skipB = {}, EditList* inv = nullptr,
                bool bases = false) {
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
        if (bases && haveR && BasisEdits(b, o, inv)) {
            ++g_basesMoved;
            o += 12;
            continue;
        }
        float raw[16];
        if (zRow || zCol) memcpy(raw, b, sizeof(raw));
        bool byColumns = false;
        if ((zRow || zCol) && !Skipped(kPartFrames) && TurnMatrix(b, MainCameraAnyFrame, &byColumns)) {
            ++g_anyFrame;
            // Only the main frame's: the sky's (the galaxy frame's) rebuild
            // their rays at the near plane, where e/near is no small change
            // (flight of 2026-09-25: the moved eye's Milky Way a smear).
            if (haveR && InMainFrame(raw)) {
                if (!InverseEdits(b, o, inv) && bases && ForwardEdits(b, o, byColumns, inv)) ++g_basesMoved;
            } else {
                ++g_invOtherFrame;
            }
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
bool TurnMask(float* f, UINT floats, EditList* inv = nullptr) {
    if (floats != 592 / 4 || Skipped(kPartMask)) return false;
    if (!SceneSized(f[0], f[1])) return false;
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
    InverseEdits(f + 4, 4, inv);  // rows 1-4: the inverse projection, in view space
    return true;
}

// Turns a mapped buffer. Map-discard memory is write-combined: reading it
// back is many times slower than ordinary memory (repeated passes over it
// cost 17 ms of CPU a frame on foot). So it is read ONCE into ordinary
// memory, matched and turned there, and written back only when turned.
bool StereoTrack(ID3D11Resource* res, float* work, UINT floats, const EditList& inv);
void Untrack(const void* res);

void TurnBuffer(float* mapped, UINT floats, ID3D11Resource* res) {
    static float work[kMaxScanBytes / 4];
    if (floats > kMaxScanBytes / 4) return;
    memcpy(work, mapped, floats * 4);
    const uint64_t before = g_anyFrame + g_scaled + g_masks;
    EditList inv{};
    TurnCopies(work, floats, {}, {}, &inv, true);
    TurnMask(work, floats, &inv);
    const bool moved = StereoTrack(res, work, floats, inv);
    if (moved || before != g_anyFrame + g_scaled + g_masks) memcpy(mapped, work, floats * 4);
}

bool PanelGBufferBound() {
    const uint32_t rg = bindingGeneration(BindSlot::Rtv0), dg = bindingGeneration(BindSlot::Dsv0);
    if (rg == g_rtvGen && dg == g_dsvGen) return g_rtvIsPanelGBuffer;
    g_rtvGen = rg;
    g_dsvGen = dg;
    g_rtvIsPanelGBuffer = false;
    g_rtvIsAnyGBuffer = false;
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
        g_rtvIsPanelGBuffer = SceneSized(td.Width, td.Height);
        g_rtvIsAnyGBuffer = td.Width >= 1024;
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
}

// A G-buffer draw's b0 as the game wrote it: a candidate for the main view.
// Any perspective (a (0, 0, 0, near) z row), normalised by its model scale.
void ConsiderMain(const float* rows) {
    const double sw = RowLength(rows + 12);
    if (sw < 1e-6) return;
    const float* z = rows + 8;
    if (std::fabs(z[0]) > 1e-4 || std::fabs(z[1]) > 1e-4 || std::fabs(z[2]) > 1e-4 || z[3] <= 1e-6) return;
    const double lx = RowLength(rows), sx = lx / sw, sy = RowLength(rows + 4) / sw;
    if (sx < 1e-6 || sy < 1e-6) return;
    if (g_haveCand && sx >= g_candScale[0]) return;
    const double xw = double(rows[0]) * rows[12] + double(rows[1]) * rows[13] + double(rows[2]) * rows[14];
    g_haveCand = true;
    g_candScale[0] = sx;
    g_candScale[1] = sy;
    g_candNear = z[3];
    g_candCentred = std::fabs(xw) < 1e-3 * lx * sw;
}

// The frame's widest, for the frames that follow.
void CommitMain() {
    if (!g_haveCand) return;
    g_haveCand = false;
    const bool changed = !g_haveMain || !Near(g_candScale[0], g_mainScale[0], 0.02 * g_mainScale[0]) ||
                         !Near(g_candNear, g_mainNear, 0.02 * g_mainNear) || g_candCentred != g_mainCentred;
    g_mainScale[0] = g_candScale[0];
    g_mainScale[1] = g_candScale[1];
    g_mainNear = g_candNear;
    g_mainCentred = g_candCentred;
    g_haveMain = true;
    if (changed && g_mainLogs < 60) {
        ++g_mainLogs;
        Log::get().note("onfoot look: main view projection x %.4f y %.4f near %.4g%s (the widest G-buffer view).",
                        g_mainScale[0], g_mainScale[1], g_mainNear, g_mainCentred ? "" : ", off-centre: not armed");
    }
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
                    "%llu timewarped (largest %.2f degrees); %llu view-model views matched to the world's field, %llu HUD views scaled "
                    "(%llu turned with the game's camera); "
                    "stereo: %s, %llu camera writes moved (%llu in view space, x%.2f), %llu written again for the other eye (%llu failed, %llu "
                    "not discards), %llu new eye targets, left eye = %s pipeline%s, half IPD %.2f mm; "
                    "head yaw %.1f pitch %.1f roll %.1f.",
                    U(g_frames), U(g_panelFrames), U(g_turnedView), U(g_turnedClip), U(g_turnedOthers),
                    U(g_otherViews), U(g_anyFrame), U(g_scaled), U(g_masks), U(g_scanned),
                    U(g_pendingFull), U(g_noPose), U(g_offCentre), U(g_ambiguous), U(g_lockDrawn), U(g_lockDeclined),
                    g_lockDeclined ? " (last: " : "", g_lockDeclined ? g_lockWhy : "", g_lockDeclined ? ")" : "",
                    U(g_lockWarped), g_lockWarpMaxDeg, U(g_modelMatched), U(g_hudScaled), U(g_hudTurned), g_stereoOn ? "on" : "off", U(g_moved), U(g_movedNear), g_nearEye,
                    U(g_rewrites), U(g_rewriteFails), U(g_notDiscard), U(g_newTargets),
                    g_leftPipe == 0 ? "first" : "second", g_leftKnown ? "" : " (assumed)", g_halfIpd * 1000,
                    g_lastYaw, g_lastPitch,
                    g_lastRoll);
    g_frames = g_panelFrames = g_turnedView = g_turnedClip = g_turnedOthers = g_otherViews = 0;
    g_anyFrame = g_scaled = g_masks = g_scanned = g_pendingFull = 0;
    g_noPose = g_offCentre = g_ambiguous = 0;
    g_lockDrawn = g_lockDeclined = g_lockWarped = g_hudScaled = g_modelMatched = g_hudTurned = 0;
    if (g_stereoOn || g_invMoved)
        Log::get().note("onfoot stereo: the lighting's eye: %llu inverse copies moved, %llu buffers kept (%llu refused, "
                        "all %d taken), %llu written again for the other eye; draws with no depth target of theirs "
                        "given an eye by the depth they read %llu, by their target %llu%s; %llu camera copies in "
                        "another world frame (the sky's) left; %llu of the lights' camera copies moved; %llu draws "
                        "given the game's own camera (onfoot_stereo_centre_vs), %llu left out (onfoot_stereo_skip_vs).",
                        U(g_invMoved), U(g_trackedCount), U(g_trackedFull), kMaxTracked, U(g_trackedRewrites),
                        U(g_pipeFromSrv), U(g_pipeFromRtv), Skipped(kPartEyeLight) ? " (part eyelight left out)" : "",
                        U(g_invOtherFrame), U(g_basesMoved), U(g_centreDraws), U(g_skippedDraws));
    g_invOtherFrame = g_basesMoved = g_centreDraws = g_skippedDraws = 0;
    g_invMoved = g_trackedFull = g_trackedRewrites = g_pipeFromSrv = g_pipeFromRtv = 0;
    g_moved =g_movedNear = g_rewrites = g_rewriteFails = g_notDiscard = g_newTargets = 0;
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


// --- the stereo diagnostic (experimental.onfoot_stereo_diag) ------------------
//
// With the engine kept in HMD stereo on foot (stereo_mode_probe.h), both eye
// pipelines render the scene, but the first flight saw the views glued to the
// face and the eyes unfusable. At each new G-buffer pass (any R10G10B10A2
// target with depth, eye-sized included) this takes b1's clip transform as last
// written -- the pass's camera -- and every two seconds logs each pass's eye:
// position (the point the clip's x, y and w rows share), axes, projection, and
// the offset between the passes in the first one's axes, with the head's yaw.
float g_lastB1Clip[16] = {};
bool g_lastB1Valid = false, g_diagWasBound = false;
constexpr int kDiagPasses = 6;
struct DiagPass {
    void* rtv;
    uint32_t w, h;
    float clip[16];
};
DiagPass g_diagPass[kDiagPasses];
int g_diagCount = 0;
ULONGLONG g_diagLastLog = 0;

bool EyeFromClip(const float* e, double c[3], double f[3], double r[3], double u[3], double* sx, double* sy,
                 double* ox, double* oy) {
    double row[4][4];
    for (int k = 0; k < 4; ++k)
        for (int i = 0; i < 4; ++i) row[k][i] = e[4 * i + k];
    const double* a = row[0];
    const double* b = row[1];
    const double* w = row[3];
    const double det = a[0] * (b[1] * w[2] - b[2] * w[1]) - a[1] * (b[0] * w[2] - b[2] * w[0]) +
                       a[2] * (b[0] * w[1] - b[1] * w[0]);
    if (std::fabs(det) < 1e-12) return false;
    const double rhs[3] = {-a[3], -b[3], -w[3]};
    auto solve = [&](int col) {
        double m[3][3] = {{a[0], a[1], a[2]}, {b[0], b[1], b[2]}, {w[0], w[1], w[2]}};
        for (int k = 0; k < 3; ++k) m[k][col] = rhs[k];
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) /
               det;
    };
    for (int i = 0; i < 3; ++i) c[i] = solve(i);
    const double lw = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    for (int i = 0; i < 3; ++i) f[i] = w[i] / lw;
    const double af = (a[0] * f[0] + a[1] * f[1] + a[2] * f[2]), bf = (b[0] * f[0] + b[1] * f[1] + b[2] * f[2]);
    for (int i = 0; i < 3; ++i) {
        r[i] = a[i] - af * f[i];
        u[i] = b[i] - bf * f[i];
    }
    *sx = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) / lw;
    *sy = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]) / lw;
    for (int i = 0; i < 3; ++i) {
        r[i] /= (*sx * lw);
        u[i] /= (*sy * lw);
    }
    *ox = af / lw;  // clip x per unit forward: the off-centre term
    *oy = bf / lw;
    return true;
}

void DiagDraw() {
    PanelGBufferBound();
    const bool bound = g_rtvIsAnyGBuffer;
    if (bound && !g_diagWasBound && g_diagCount < kDiagPasses && g_lastB1Valid) {
        DiagPass& d = g_diagPass[g_diagCount++];
        d.rtv = bindingGet(BindSlot::Rtv0);
        d.w = g_lastGbufW;
        d.h = g_lastGbufH;
        memcpy(d.clip, g_lastB1Clip, sizeof(d.clip));
    }
    g_diagWasBound = bound;
}

void DiagFrame() {
    const ULONGLONG now = GetTickCount64();
    if (g_diagCount && now - g_diagLastLog > 2000) {
        g_diagLastLog = now;
        float m[12];
        double yaw = 0, pitch = 0;
        if (headPose(m)) {
            yaw = std::atan2(-m[2], m[10]) * 57.29577951308232;
            pitch = std::asin(std::fmax(-1.0, std::fmin(1.0, m[6]))) * 57.29577951308232;
        }
        double c0[3] = {}, f0[3] = {}, r0[3] = {}, u0[3] = {};
        for (int k = 0; k < g_diagCount; ++k) {
            const DiagPass& d = g_diagPass[k];
            double c[3], f[3], r[3], u[3], sx, sy, ox, oy;
            if (!EyeFromClip(d.clip, c, f, r, u, &sx, &sy, &ox, &oy)) {
                Log::get().note("onfoot stereo diag: pass %d (%ux%u) has no eye point.", k, d.w, d.h);
                continue;
            }
            if (k == 0) {
                memcpy(c0, c, sizeof(c0));
                memcpy(f0, f, sizeof(f0));
                memcpy(r0, r, sizeof(r0));
                memcpy(u0, u, sizeof(u0));
            }
            const double dc[3] = {c[0] - c0[0], c[1] - c0[1], c[2] - c0[2]};
            const double dr = dc[0] * r0[0] + dc[1] * r0[1] + dc[2] * r0[2];
            const double du = dc[0] * u0[0] + dc[1] * u0[1] + dc[2] * u0[2];
            const double df = dc[0] * f0[0] + dc[1] * f0[1] + dc[2] * f0[2];
            const double cosF = std::fmax(-1.0, std::fmin(1.0, f[0] * f0[0] + f[1] * f0[1] + f[2] * f0[2]));
            Log::get().note("onfoot stereo diag: pass %d rtv %p %ux%u: eye (%.4f %.4f %.4f) forward (%.3f %.3f %.3f) "
                            "right (%.3f %.3f %.3f); proj sx %.3f sy %.3f off %.3f %.3f; vs pass 0: offset right "
                            "%.4f up %.4f fwd %.4f, forward differs %.2f deg; head yaw %.1f pitch %.1f.",
                            k, d.rtv, d.w, d.h, c[0], c[1], c[2], f[0], f[1], f[2], r[0], r[1], r[2], sx, sy, ox, oy,
                            dr, du, df, std::acos(cosF) * 57.29577951308232, yaw, pitch);
        }
    }
    g_diagCount = 0;
    g_diagWasBound = false;
}

// --- the on-foot stereo's eyes: the work ------------------------------------

// The eye offset of a pipeline along the camera's right axis, metres.
// experimental.onfoot_stereo_anchor: which point of the head the game's
// camera stands for. The centre (0) puts the eyes half the IPD either side
// of it. The game aims down sights by putting the sights on its camera, so
// with the centre the sights sit before the nose; anchored on the right eye
// (+1) or the left (-1), that eye is the camera -- it sees exactly what the
// flat game shows -- and the other is a whole IPD away.
int g_anchor = 0;
// experimental.onfoot_stereo_centre_vs: draws by these vertex shaders take
// the game's own camera in both eyes (what is at infinity: the eyes must not
// part it). Found with the capture's draw records.
constexpr int kPipeCentre = 2;
uint64_t g_centreVs[16] = {};
int g_centreVsCount = 0;
// experimental.onfoot_stereo_skip_vs: draws by these vertex shaders left out
// (a developer switch: which effect a difference between the eyes follows).
uint64_t g_skipVs[16] = {};
int g_skipVsCount = 0;

int ParseHashes(const std::string& list, uint64_t out[16]) {
    int count = 0;
    for (size_t at = 0; at < list.size() && count < 16;) {
        const size_t end = list.find(',', at);
        const std::string item = list.substr(at, end == std::string::npos ? std::string::npos : end - at);
        char* stop = nullptr;
        const uint64_t v = _strtoui64(item.c_str(), &stop, 16);
        if (v) out[count++] = v;
        if (end == std::string::npos) break;
        at = end + 1;
    }
    return count;
}


double PipeOffset(int pipe) {
    if (pipe == kPipeCentre) return 0;
    const double base = pipe == g_leftPipe ? -g_halfIpd : g_halfIpd;
    return base - g_anchor * g_halfIpd;
}

// rows (by rows, turned and matched): sx to move by, or 0 when not a view
// of this camera -- a centred perspective with the main aspect, whatever its
// near plane and field: the world, the same camera through a model, the view
// model, the HUD and the holograms, all from the same eye. (The first flights
// moved only the main near plane and the helmet HUD's; the gun's ammo
// hologram, drawn at a 0.1 near plane through the world's field, stayed at
// the centre in both eyes.) Part "hudeye" leaves those off the main near
// plane at the centre.
double MoveScale(const float* rows) {
    if (!g_haveMain) return 0;
    const double sx = RowLength(rows), sy = RowLength(rows + 4), sw = RowLength(rows + 12);
    if (sx < 1e-6 || sy < 1e-6 || sw < 1e-6) return 0;
    const double aspect = g_mainScale[0] / g_mainScale[1];
    if (!Near(sx / sy, aspect, 5e-3 * aspect)) return 0;
    bool finite = false;
    if (!PerspectiveZ(rows, &finite)) return 0;
    if (Skipped(kPartHudEye) && !Near(rows[11], g_mainNear, 0.02 * g_mainNear)) return 0;
    const double dot = double(rows[0]) * rows[12] + double(rows[1]) * rows[13] + double(rows[2]) * rows[14];
    if (std::fabs(dot) > 1e-3 * sx * sw) return 0;
    return sx / sw;
}

int PredictPipe() { return g_curPipe >= 0 ? g_curPipe : (g_lastPipe >= 0 ? g_lastPipe : 0); }

// b0 as the game wrote it and the head look turned it (mapped, write-combined;
// rows the turned rows already read out): kept, and moved for a pipeline.
void ApplyEdits(float* dst, const float* copy, const EditList& l, int pipe);

void StereoViewWritten(float* mapped, const float* rows, bool viewSpace) {
    g_b0Edits.n = 0;
    if (!g_stereoOn || g_viewBytes > sizeof(g_b0Copy)) return;
    double sx = MoveScale(rows);
    if (sx <= 0) return;
    if (viewSpace) {
        sx *= g_nearEye;
        ++g_movedNear;
    }
    if (!g_viewDiscard) {
        ++g_notDiscard;
        return;
    }
    memcpy(g_b0Copy, mapped, g_viewBytes);
    g_b0CopyBytes = g_viewBytes;
    g_b0Edits.e[0] = {kB0Moved, static_cast<float>(-sx)};
    g_b0Edits.n = 1;
    g_b0Pipe = PredictPipe();
    ApplyEdits(mapped, g_b0Copy, g_b0Edits, g_b0Pipe);
    ++g_moved;
}

// b1 in ordinary memory, turned, before it is written back.
void StereoFrameWritten(float* work, bool viewSpace, const EditList& inv) {
    g_b1Edits.n = 0;
    if (!g_stereoOn) return;
    const float* c = work + kClipOffset / 4;
    float rows[16];
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) rows[4 * r + col] = c[4 * col + r];
    double sx = MoveScale(rows);
    if (sx > 0) {
        if (viewSpace) {
            sx *= g_nearEye;
            ++g_movedNear;
        }
        g_b1Edits.e[g_b1Edits.n++] = {kB1Moved, static_cast<float>(-sx)};
    }
    const int invCount = Skipped(kPartEyeLight) ? 0 : inv.n;
    for (int i = 0; i < invCount && g_b1Edits.n < kMaxEdits; ++i) g_b1Edits.e[g_b1Edits.n++] = inv.e[i];
    if (!g_b1Edits.n) return;
    if (!g_frameDiscard) {
        ++g_notDiscard;
        g_b1Edits.n = 0;
        return;
    }
    memcpy(g_b1Copy, work, g_frameFloats * 4);
    g_b1CopyBytes = g_frameFloats * 4;
    g_b1Pipe = PredictPipe();
    ApplyEdits(work, g_b1Copy, g_b1Edits, g_b1Pipe);
    if (sx > 0) ++g_moved;
    g_invMoved += invCount / 3;
}

// The pipeline whose depth target this draw uses; -1 for none of theirs.
int DsvPipe() {
    const uint32_t dg = bindingGeneration(BindSlot::Dsv0);
    if (dg == g_pipeDsvGen) return g_curPipe;
    g_pipeDsvGen = dg;
    g_curPipe = -1;
    auto* dsv = static_cast<ID3D11DepthStencilView*>(bindingGet(BindSlot::Dsv0));
    if (!dsv) return -1;
    ID3D11Resource* res = nullptr;
    dsv->GetResource(&res);
    if (!res) return -1;
    for (int i = 0; i < g_pipeTargetCount; ++i)
        if (g_pipeTargets[i].res == res) g_curPipe = g_pipeTargets[i].pipe;
    if (g_curPipe < 0) {
        ID3D11Texture2D* tex = nullptr;
        if (SUCCEEDED(res->QueryInterface(__uuidof(ID3D11Texture2D), reinterpret_cast<void**>(&tex))) && tex) {
            D3D11_TEXTURE2D_DESC td;
            tex->GetDesc(&td);
            tex->Release();
            if (SceneSized(td.Width, td.Height)) {
                if (g_pipeTargetCount == kMaxPipeTargets) g_pipeTargetCount = 0;
                g_curPipe = g_lastPipe >= 0 ? 1 - g_lastPipe : 0;
                g_pipeTargets[g_pipeTargetCount++] = {res, g_curPipe};
                ++g_newTargets;
                g_fbGen[0] = 0;  // the fallback's answer may change
            }
        }
    }
    res->Release();
    if (g_curPipe >= 0) g_lastPipe = g_curPipe;
    return g_curPipe;
}

// The draw's pipeline: its depth target's; else (a lighting pass: no depth
// target, the depth read as a texture) the pipeline whose depth it samples in
// pixel slots 0-3; else whose render target (seen this frame under that
// pipeline's depth) it draws into.
int PipeOfDraw(int* src) {
    const int d = DsvPipe();
    if (d >= 0) {
        *src = kFromDsv;
        return d;
    }
    static const BindSlot kSlots[5] = {BindSlot::PsSrv0, BindSlot::PsSrv1, BindSlot::PsSrv2, BindSlot::PsSrv3,
                                       BindSlot::Rtv0};
    bool same = true;
    for (int i = 0; i < 5; ++i) {
        const uint32_t g = bindingGeneration(kSlots[i]);
        if (g != g_fbGen[i]) {
            g_fbGen[i] = g;
            same = false;
        }
    }
    if (!same) {
        g_fbPipe = -1;
        g_fbSrc = kFromNone;
        for (int i = 0; i < 4 && g_fbPipe < 0; ++i) {
            void* v = bindingGet(kSlots[i]);
            ResourceInfo info{};
            if (!v || !bindingResolve(v, &info) || !info.resource) continue;
            for (int t = 0; t < g_pipeTargetCount; ++t)
                if (g_pipeTargets[t].res == info.resource) {
                    g_fbPipe = g_pipeTargets[t].pipe;
                    g_fbSrc = kFromSrv;
                }
        }
        void* rtv = bindingGet(BindSlot::Rtv0);
        ResourceInfo info{};
        if (g_fbPipe < 0 && rtv && bindingResolve(rtv, &info) && info.resource)
            for (int p = 0; p < 2; ++p)
                for (int i = 0; i < g_pipeRtvCount[p]; ++i)
                    if (g_pipeRtvs[p][i] == info.resource) {
                        g_fbPipe = p;
                        g_fbSrc = kFromRtv;
                    }
    }
    *src = g_fbSrc;
    return g_fbPipe;
}

// The render target of a draw of a pipeline, remembered for the left-eye match.
void NotePipeTarget(int pipe) {
    const uint32_t rg = bindingGeneration(BindSlot::Rtv0);
    if (rg == g_pipeRtvGen) return;
    g_pipeRtvGen = rg;
    auto* rtv = static_cast<ID3D11RenderTargetView*>(bindingGet(BindSlot::Rtv0));
    if (!rtv) return;
    ID3D11Resource* res = nullptr;
    rtv->GetResource(&res);
    if (!res) return;
    res->Release();  // compared by address only
    for (int i = 0; i < g_pipeRtvCount[pipe]; ++i)
        if (g_pipeRtvs[pipe][i] == res) return;
    if (g_pipeRtvCount[pipe] < 8) g_pipeRtvs[pipe][g_pipeRtvCount[pipe]++] = res;
}

void ApplyEdits(float* dst, const float* copy, const EditList& l, int pipe) {
    const double offset = PipeOffset(pipe);
    for (int i = 0; i < l.n; ++i) dst[l.e[i].index] = static_cast<float>(copy[l.e[i].index] + l.e[i].coef * offset);
}

void Rewrite(ID3D11DeviceContext* ctx, ID3D11Buffer* buf, const float* copy, UINT bytes, const EditList& edits,
             int pipe) {
    D3D11_MAPPED_SUBRESOURCE m{};
    if (!g_realMap || !g_realUnmap || !buf || FAILED(g_realMap(ctx, buf, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)) ||
        !m.pData) {
        ++g_rewriteFails;
        return;
    }
    memcpy(m.pData, copy, bytes);
    ApplyEdits(static_cast<float*>(m.pData), copy, edits, pipe);
    g_realUnmap(ctx, buf, 0);
    ++g_rewrites;
}

Tracked* FindTracked(const void* res) {
    for (int i = 0; i < g_trackedCount; ++i)
        if (g_tracked[i].buf == res) return &g_tracked[i];
    return nullptr;
}

void Untrack(const void* res) {
    Tracked* t = FindTracked(res);
    if (!t) return;
    t->buf->Release();
    Tracked& last = g_tracked[--g_trackedCount];
    if (t != &last) {
        t->buf = last.buf;
        t->bytes = last.bytes;
        t->pipe = last.pipe;
        t->edits = last.edits;
        memcpy(t->copy, last.copy, last.bytes);
    }
}

void UntrackAll() {
    for (int i = 0; i < g_trackedCount; ++i) g_tracked[i].buf->Release();
    g_trackedCount = 0;
}

// A scanned buffer as turned (work): with the camera's inverses in it, kept
// and moved for the pipeline that draws next; without, no longer ours.
bool StereoTrack(ID3D11Resource* res, float* work, UINT floats, const EditList& inv) {
    if (!g_stereoOn || !inv.n || Skipped(kPartEyeLight) || !res) {
        Untrack(res);
        return false;
    }
    Tracked* t = FindTracked(res);
    if (!t) {
        if (g_trackedCount == kMaxTracked) {
            ++g_trackedFull;
            return false;
        }
        t = &g_tracked[g_trackedCount++];
        t->buf = static_cast<ID3D11Buffer*>(res);
        t->buf->AddRef();
    }
    t->bytes = floats * 4;
    t->edits = inv;
    memcpy(t->copy, work, t->bytes);
    t->pipe = PredictPipe();
    ApplyEdits(work, t->copy, t->edits, t->pipe);
    g_invMoved += inv.n / 3;
    return true;
}

void StereoWriteFor(ID3D11DeviceContext* ctx, int pipe);

void StereoDraw(ID3D11DeviceContext* ctx) {
    int src = kFromNone;
    const int pipe = PipeOfDraw(&src);
    g_drawPipe = pipe;
    g_drawPipeSrc = src;
    if (pipe < 0) return;
    if (src == kFromDsv) NotePipeTarget(pipe);
    else if (src == kFromSrv) ++g_pipeFromSrv;
    else ++g_pipeFromRtv;
    int want = pipe;
    if (g_centreVsCount) {
        const uint64_t vs = bindingShaderHash(BindSlot::Vs);
        for (int i = 0; i < g_centreVsCount; ++i)
            if (g_centreVs[i] == vs) {
                want = kPipeCentre;
                ++g_centreDraws;
            }
    }
    StereoWriteFor(ctx, want);
}

void StereoWriteFor(ID3D11DeviceContext* ctx, int pipe) {
    if (g_b0Edits.n && g_b0Pipe != pipe) {
        Rewrite(ctx, g_viewCb, g_b0Copy, g_b0CopyBytes, g_b0Edits, pipe);
        g_b0Pipe = pipe;
    }
    if (g_b1Edits.n && g_b1Pipe != pipe) {
        Rewrite(ctx, g_frameCb, g_b1Copy, g_b1CopyBytes, g_b1Edits, pipe);
        g_b1Pipe = pipe;
    }
    for (int i = 0; i < g_trackedCount; ++i) {
        Tracked& t = g_tracked[i];
        if (t.pipe == pipe) continue;
        Rewrite(ctx, t.buf, t.copy, t.bytes, t.edits, pipe);
        t.pipe = pipe;
        ++g_trackedRewrites;
    }
}

void StereoForget() {
    g_pipeTargetCount = 0;
    g_curPipe = g_lastPipe = -1;
    g_pipeDsvGen = g_pipeRtvGen = 0;
    g_leftKnown = g_leftGuessNoted = false;
    g_leftPipe = 0;
    g_b0Edits.n = g_b1Edits.n = 0;
    UntrackAll();
    for (uint32_t& g : g_fbGen) g = 0;
}

// The frame that ended: which pipeline the game submitted as the left eye.
// Then the coming frame: on or off, the half IPD, the flat frustum published.
void StereoFrameBoundary() {
    if (g_stereoOn) {
        ++g_stereoFrames;
        void* left = gameSubmitted(0);
        void* right = gameSubmitted(1);
        int leftPipe = -1;
        for (int p = 0; p < 2; ++p)
            for (int i = 0; i < g_pipeRtvCount[p]; ++i) {
                if (left && g_pipeRtvs[p][i] == left) leftPipe = p;
                if (right && g_pipeRtvs[p][i] == right) leftPipe = 1 - p;
            }
        if (leftPipe >= 0 && (!g_leftKnown || leftPipe != g_leftPipe)) {
            Log::get().note("onfoot stereo: the %s pipeline renders the left eye (its target is the texture the game "
                            "submits for it).",
                            leftPipe == 0 ? "first" : "second");
            g_leftPipe = leftPipe;
            g_leftKnown = true;
        }
        if (!g_leftKnown && !g_leftGuessNoted && g_stereoFrames > 300) {
            g_leftGuessNoted = true;
            Log::get().note("onfoot stereo: no pipeline's target is a submitted texture (left %p, right %p); the first "
                            "pipeline is taken as the left eye (onfoot_stereo_swap_eyes crosses them).",
                            left, right);
        }
    }
    g_pipeRtvCount[0] = g_pipeRtvCount[1] = 0;
    g_pipeRtvGen = 0;
    g_b0Edits.n = g_b1Edits.n = 0;  // last frame's writes
    for (uint32_t& g : g_fbGen) g = 0;

    const bool want = onFootStereoWanted() && g_panelLastFrame && g_haveMain;
    const double ipd = g_ipdMm > 0 ? g_ipdMm / 1000.0 : (g_ipdMm == 0 ? double(eyeSeparation()) : 0.0);
    const bool on = want;
    if (on != g_stereoWasOn) {
        if (on)
            Log::get().note("onfoot stereo: eyes %s (IPD %.1f mm at the start, %s); each image placed at the flat frustum's "
                            "angles (x %.3f, y %.3f).",
                            g_ipdMm >= 0 ? "moved" : "NOT moved", ipd * 1000,
                            g_ipdMm > 0 ? "onfoot_stereo_ipd_mm" : (g_ipdMm == 0 ? "the headset's" : "off"),
                            1 / g_mainScale[0], 1 / g_mainScale[1]);
        else
            Log::get().note("onfoot stereo: off (%llu frames).", static_cast<unsigned long long>(g_stereoFrames));
        StereoForget();
        g_stereoFrames = 0;
        g_stereoWasOn = on;
    }
    g_stereoOn = on;
    g_halfIpd = ipd / 2;
    setOnFootFlat(on, on ? static_cast<float>(1 / g_mainScale[0]) : 0.0f,
                  on ? static_cast<float>(1 / g_mainScale[1]) : 0.0f);
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

// A draw, after the stereo's writes for it: its pipeline (and how it was
// told), which pipeline b0 and b1 hold (-1: not moved), its shaders, targets,
// the constant buffers bound in slots 0-7 of both stages and the pixel
// shader's resources 0-3, by the capture's buffer ids. Buffer id 0xFFFFFFFE.
void CapDraw(ID3D11DeviceContext* ctx) {
    struct {
        uint32_t ordinal;
        int32_t pipe, src, b0Pipe, b1Pipe;
        uint32_t rtv, dsv, tracked;
        uint64_t vs, ps;
        uint32_t vsCb[8], psCb[8], psSrv[4];
    } d{};
    d.ordinal = g_drawOrdinal;
    d.pipe = g_stereoOn ? g_drawPipe : -2;
    d.src = g_drawPipeSrc;
    d.b0Pipe = g_b0Edits.n ? g_b0Pipe : -1;
    d.b1Pipe = g_b1Edits.n ? g_b1Pipe : -1;
    d.tracked = static_cast<uint32_t>(g_trackedCount);
    ResourceInfo info{};
    void* rtv = bindingGet(BindSlot::Rtv0);
    if (rtv && bindingResolve(rtv, &info) && info.resource) d.rtv = CapId(info.resource);
    info = {};
    void* dsv = bindingGet(BindSlot::Dsv0);
    if (dsv && bindingResolve(dsv, &info) && info.resource) d.dsv = CapId(info.resource);
    d.vs = bindingShaderHash(BindSlot::Vs);
    d.ps = bindingShaderHash(BindSlot::Ps);
    ID3D11Buffer* cbs[8] = {};
    ctx->VSGetConstantBuffers(0, 8, cbs);
    for (int i = 0; i < 8; ++i)
        if (cbs[i]) {
            d.vsCb[i] = CapId(cbs[i]);
            cbs[i]->Release();
            cbs[i] = nullptr;
        }
    ctx->PSGetConstantBuffers(0, 8, cbs);
    for (int i = 0; i < 8; ++i)
        if (cbs[i]) {
            d.psCb[i] = CapId(cbs[i]);
            cbs[i]->Release();
        }
    ID3D11ShaderResourceView* srvs[4] = {};
    ctx->PSGetShaderResources(0, 4, srvs);
    for (int i = 0; i < 4; ++i)
        if (srvs[i]) {
            ID3D11Resource* r = nullptr;
            srvs[i]->GetResource(&r);
            if (r) {
                d.psSrv[i] = CapId(r);
                r->Release();
            }
            srvs[i]->Release();
        }
    CapRecord(0xFFFFFFFEu, &d, sizeof(d));
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
    const bool diag = cfg.getBool("experimental.onfoot_stereo_diag", false);
    if (diag != g_stereoDiag)
        Log::get().note("onfoot look: stereo diagnostic %s.", diag ? "ON (each G-buffer pass's eye logged)" : "off");
    g_stereoDiag = diag;
    g_rtvGen = g_dsvGen = 0;
    const float hud = cfg.getFloat("experimental.onfoot_hud_scale", 1.0f);
    const float hudClamped = hud < 0.3f ? 0.3f : (hud > 1.5f ? 1.5f : hud);
    if (hudClamped != g_hudScale) Log::get().note("onfoot look: the HUD drawn at %.2f of its size.", hudClamped);
    g_hudScale = hudClamped;
    const bool match = cfg.getBool("experimental.onfoot_match_fov", true);
    if (match != g_matchFov)
        Log::get().note("onfoot look: the view model (body, what it holds) drawn %s.",
                        match ? "through the world's projection" : "as the game draws it");
    g_matchFov = match;
    cameraHuntConfigure(cfg);
    memProbeConfigure(cfg);
    headDriveConfigure(cfg);
    const std::string skip = cfg.getString("experimental.onfoot_head_look_skip", "");
    unsigned mask = 0;
    const struct {
        const char* name;
        Part part;
    } kParts[] = {{"view", kPartView},     {"others", kPartOthers},     {"mask", kPartMask},
                  {"frames", kPartFrames}, {"scaled", kPartScaled},     {"timewarp", kPartTimewarp},
                  {"hudeye", kPartHudEye}, {"eyelight", kPartEyeLight}};
    for (const auto& p : kParts)
        if (skip.find(p.name) != std::string::npos) mask |= p.part;
    if (mask != g_skip) Log::get().note("onfoot look: parts left out: \"%s\" (mask %u).", skip.c_str(), mask);
    g_skip = mask;
    const float ipd = cfg.getFloat("experimental.onfoot_stereo_ipd_mm", 0.0f);
    if (ipd != g_ipdMm)
        Log::get().note("onfoot stereo: IPD %s.", ipd > 0 ? "from onfoot_stereo_ipd_mm" : (ipd == 0 ? "the headset's" : "none: the eyes are not moved"));
    g_ipdMm = ipd;
    const std::string centre = cfg.getString("experimental.onfoot_stereo_centre_vs", "");
    uint64_t centreVs[16] = {};
    const int centreCount = ParseHashes(centre, centreVs);
    if (centreCount != g_centreVsCount || memcmp(centreVs, g_centreVs, sizeof(centreVs)) != 0)
        Log::get().note("onfoot stereo: %d vertex shader(s) drawn from the game's own camera in both eyes (\"%s\").",
                        centreCount, centre.c_str());
    memcpy(g_centreVs, centreVs, sizeof(centreVs));
    g_centreVsCount = centreCount;
    const std::string skipList = cfg.getString("experimental.onfoot_stereo_skip_vs", "");
    uint64_t skipVs[16] = {};
    const int skipCount = ParseHashes(skipList, skipVs);
    if (skipCount != g_skipVsCount || memcmp(skipVs, g_skipVs, sizeof(skipVs)) != 0)
        Log::get().note("onfoot stereo: %d vertex shader(s) left out (\"%s\").", skipCount, skipList.c_str());
    memcpy(g_skipVs, skipVs, sizeof(skipVs));
    g_skipVsCount = skipCount;
    const float nearEye = cfg.getFloat("experimental.onfoot_stereo_near_eye", 1.0f);
    const double nearClamped = nearEye < 0 ? 0.0 : (nearEye > 1 ? 1.0 : nearEye);
    if (nearClamped != g_nearEye)
        Log::get().note("onfoot stereo: view-space draws (the body, what it holds) take %.2f of the eye offset.",
                        nearClamped);
    g_nearEye = nearClamped;
    const std::string anchorName = cfg.getString("experimental.onfoot_stereo_anchor", "centre");
    const int anchor = anchorName == "right" ? 1 : (anchorName == "left" ? -1 : 0);
    if (anchor != g_anchor)
        Log::get().note("onfoot stereo: the game's camera is your %s.",
                        anchor > 0 ? "right eye" : (anchor < 0 ? "left eye" : "head's centre, between the eyes"));
    g_anchor = anchor;
    if (!on) {
        if (g_stereoWasOn) setOnFootFlat(false, 0, 0);
        g_stereoOn = g_stereoWasOn = false;
        StereoForget();
        g_viewData = g_frameData = nullptr;
        g_panelLastFrame = false;
    }
}

void onFootLookSetMapFns(OnFootMapFn map, OnFootUnmapFn unmap) {
    g_realMap = map;
    g_realUnmap = unmap;
}

bool onFootLookBeforeDraw(ID3D11DeviceContext* ctx) {
    if (!detail::g_onFootLookEnabled) return false;
    ++g_drawOrdinal;
    if (g_stereoDiag) DiagDraw();
    if (g_stereoOn && ctx) StereoDraw(ctx);
    if (g_capFile && ctx) CapDraw(ctx);
    bool skip = false;
    if (g_stereoOn && g_skipVsCount) {
        const uint64_t vs = bindingShaderHash(BindSlot::Vs);
        for (int i = 0; i < g_skipVsCount; ++i) skip |= g_skipVs[i] == vs;
        if (skip) ++g_skippedDraws;
    }
    if (!PanelGBufferBound()) return skip;
    if (g_viewRawFresh && bindingGet(BindSlot::VsCb0) == g_viewCb) {
        g_viewRawFresh = false;
        ConsiderMain(g_viewRaw);
    }
    if (g_foundThisFrame) return skip;
    g_foundThisFrame = true;
    Learn();
    return skip;
}

void onFootLookMapped(ID3D11Resource* res, void* data, D3D11_MAP type) {
    if (!detail::g_onFootLookEnabled || type == D3D11_MAP_READ || !res || res == g_lockCb) return;
    if (res == g_viewCb) {
        g_viewData = data;
        g_viewDiscard = type == D3D11_MAP_WRITE_DISCARD;
        return;
    }
    if (res == g_frameCb) {
        g_frameData = data;
        g_frameDiscard = type == D3D11_MAP_WRITE_DISCARD;
        return;
    }
    // Any other small constant or shader-resource buffer rewritten whole
    // while the panel scene runs: a candidate for camera copies.
    // A kept copy the game writes past (not whole, or not seen) is stale.
    if (!g_panelLastFrame || type != D3D11_MAP_WRITE_DISCARD) {
        if (g_trackedCount) Untrack(res);
        return;
    }
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
    if (g_trackedCount) Untrack(res);
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
        float before[16];
        memcpy(before, a, sizeof(before));
        memcpy(g_viewRaw, before, sizeof(g_viewRaw));
        g_viewRawFresh = true;
        TurnView(a);
        memcpy(g_viewShadow, a, sizeof(g_viewShadow));
        g_viewShadowFresh = true;
        StereoViewWritten(static_cast<float*>(g_viewData), g_viewShadow, g_stereoOn && ViewSpaceRows(before));
        g_viewData = nullptr;
    } else if (res == g_frameCb && g_frameData) {
        // The view slot (4320) is turned only for the main view; every b1
        // write also carries the camera's own copies (R^T at 3664, R at 4432,
        // the sky's inverse at 560), whichever view the slot holds. Worked on
        // in ordinary memory and written back whole (see TurnBuffer).
        static float work[kMaxScanBytes / 4];
        memcpy(work, g_frameData, g_frameFloats * 4);
        if (!g_rawCamValid && g_frameFloats * 4 >= kCamRowsOffset + 48) {
            float clip[16];
            const float* c = work + kClipOffset / 4;
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col) clip[4 * r + col] = c[4 * col + r];
            if (IsMainView(clip)) {
                memcpy(g_rawCam, work + kCamRowsOffset / 4, sizeof(g_rawCam));
                g_rawCamValid = true;
            }
        }
        bool viewSpace = false;
        if (g_stereoOn) {
            float rows[16];
            const float* c = work + kClipOffset / 4;
            for (int r = 0; r < 4; ++r)
                for (int col = 0; col < 4; ++col) rows[4 * r + col] = c[4 * col + r];
            viewSpace = ViewSpaceRows(rows);
        }
        TurnClip(work + kClipOffset / 4);
        EditList inv{};
        if (Armed())
            TurnCopies(work, g_frameFloats, {kClipOffset / 4, kClipOffset / 4 + 16},
                       {kPrevPoseOffset / 4, kPrevPoseOffset / 4 + 12}, &inv);
        StereoFrameWritten(work, viewSpace, inv);
        memcpy(g_frameData, work, g_frameFloats * 4);
        memcpy(g_eyeClip, work + kClipOffset / 4, sizeof(g_eyeClip));
        g_eyeClipValid = true;
        memcpy(g_lastB1Clip, work + kClipOffset / 4, sizeof(g_lastB1Clip));
        g_lastB1Valid = true;
        g_frameData = nullptr;
    } else {
        for (Pending& p : g_pending) {
            if (p.res != res) continue;
            if (Armed()) TurnBuffer(p.data, p.floats, p.res);
            else Untrack(p.res);
            p = {};
            break;
        }
    }
}

void onFootLookStateCleared() {
    g_rtvGen = g_dsvGen = 0;
    g_pipeDsvGen = g_pipeRtvGen = 0;
    g_rtvIsPanelGBuffer = false;
}

void onFootLookFrameBoundary() {
    memProbeFrame();
    headDriveFrame();
    g_driveTaken = false;
    if (!detail::g_onFootLookEnabled) return;
    ++g_frames;
    if (g_stereoDiag) DiagFrame();
    if (g_foundThisFrame) ++g_panelFrames;
    g_panelLastFrame = g_foundThisFrame;
    g_foundThisFrame = false;
    CommitMain();
    cameraHuntFrame(g_rawCamValid && g_panelLastFrame && (!onFootStereoHolding() || onFootStereoWanted()) ? g_rawCam
                                                                                                          : nullptr);
    g_rawCamValid = false;
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
    StereoFrameBoundary();
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
