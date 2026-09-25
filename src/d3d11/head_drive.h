// The head drives the game's own on-foot look (experimental.onfoot_head_drive).
//
// WHY THIS EXISTS
//
// The head look (onfoot_look.h) turns the picture, not the game: the game
// aims, culls and walks along its own camera, which only the stick moves
// (flight of 2026-09-24: look up 45 degrees with the head and the shots still
// go level). Here the head moves the game's camera itself, so aim, culling
// and the walking direction follow the head, and the head look is left with
// the difference: the head's roll, pitch past the game's limits, and the
// frames the game's camera is behind the head.
//
// WHERE THE LOOK LIVES (build 332841; the camera hunt and the probes of
// 2026-09-24, docs/cobra-onfoot-frame.md)
//
// The player's look component Y (vtable +0x52037A8) holds the body's frame
// at Y+0x5B0 (right, up, forward: float4 rows; up is the local vertical) and
// the look pitch at Y+0x718 (radians, positive DOWN). The camera is built
// from the body frame turned by that pitch (less the recoil at Y+0x738) and a
// small yaw offset. The input update (+0x1A9C88B...) computes each frame
// pitch - the stick's pitch input, clamps it to the look's limits, and
// stores it:
//
//     +0x1A9CB11  41 0F 2F C0      comiss xmm0,xmm8   ; the lower limit
//     +0x1A9CB15  77 08            ja     +0x1A9CB1F
//     +0x1A9CB17  0F 28 C6         movaps xmm0,xmm6   ; the upper limit
//     +0x1A9CB1A  F3 41 0F 5D C0   minss  xmm0,xmm8
//     +0x1A9CB1F  movss [rdi+71Ch],xmm0; movss [rdi+718h],xmm0
//
// The hook replaces the comiss/ja with a jump to a stub that calls in with
// Y (rdi) and the candidate pitch (xmm8): the head's pitch goes in its place
// and the game clamps it to its own limits. On the same call, on the game's
// own thread, the body frame is turned about its up row by the head's yaw
// since the last frame (as the game's stick turn does, +0x1A9D014); the stick
// still turns the body on top of it.
//
// The render side needs to know which head sample the camera it draws was
// built from (the game's camera runs a frame or two behind the head). Each
// game frame records the yaw it applied with the body frame the frame left;
// headDriveCamera matches the drawn camera's heading against those records.
//
// Refuses unless the executable is build 332841 and the bytes are the ones
// read from it. Installed at the first configure with it on; off (live) puts
// the stick back in charge of the pitch.
#pragma once

namespace edvr {

class Config;

void headDriveConfigure(Config& cfg);

// Once a frame at the render thread's frame boundary: the report.
void headDriveFrame();

// Render thread. axes: the drawn main view's world axes by rows (right, up,
// forward), as the game wrote them. qGame: that camera as a rotation in the
// head's tracking frame, in the head look's form (game view axes; yaw about
// y, positive right; pitch about x, positive down): the yaw of the head
// sample the game applied for it, its pitch as the game clamped it. False
// when the drive is not driving this camera (off, not installed, no record
// matches): the head look then turns by the whole head.
bool headDriveCamera(const double axes[3][3], double qGame[3][3]);

// The head drove a game frame within the last half second: the drawn camera
// has the head in it even when headDriveCamera cannot say which sample, so
// the head look must not turn by the whole head again.
bool headDriveActive();

// The residual the head look turned the frame's main view by, in degrees
// (the report's average).
void headDriveNoteResidual(double degrees);

}  // namespace edvr
