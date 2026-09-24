// Where the game keeps the on-foot camera: a hunt (experimental.onfoot_camera_hunt).
//
// WHY THIS EXISTS
//
// The head look turns the picture, not the game's camera: the game aims, and
// culls, where its own camera points, which the stick moves and the head does
// not (flight of 2026-09-24: look up 45 degrees and the shots still go level).
// The fix is the head driving the game's look angles. This finds them.
//
// Once on foot with the camera still for a second, the process's private
// read-write memory is searched for the camera's forward row as the game
// wrote it into b1 (cb1 at 4432: right, up, forward) -- as floats and as
// doubles -- and every hit is logged with its neighbourhood. Then the hits
// that follow the camera as it turns are kept, and hardware write-watches
// (DR1-DR3, three at a time, three seconds each) name the code that writes
// each one, with the game's stack: the camera update, which reads the look
// angles. A developer instrument; off by default; it suspends every thread
// for a moment every two seconds while watching.
#pragma once

namespace edvr {

class Config;

void cameraHuntConfigure(Config& cfg);

// Once a frame, on foot with the head look on: the frame's camera rows as the
// game wrote them (right, up, forward: 3 x float4, w unused), or null when
// the frame had none.
void cameraHuntFrame(const float* raw12);

// While it holds the debug registers (hw_watch.h has one user at a time).
bool cameraHuntWatching();

}  // namespace edvr
