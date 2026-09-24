// Live probes on the game's memory, from the ini while the game runs
// (experimental.probe_*): a developer instrument.
//
// WHY THIS EXISTS
//
// The camera hunt (camera_hunt.h) names where the game keeps its on-foot
// camera and the code that writes it; the look angles sit further up that
// chain, and each step up (what does this object point to, who writes that
// field) used to cost a build and a flight. These keys take the step live:
//
//   probe_peek  = <expr> [len]; ...        dump len bytes (hex, default 100)
//                                           at each address: qwords, floats,
//                                           plausible doubles, and what a
//                                           qword points to (image RVA, or an
//                                           object's vtable RVA)
//   probe_watch = <expr> [1|2|4|8] [rw]; ...  hardware watch (DR1-DR3, up to
//                                           three) for probe_watch_ms: every
//                                           writer (rw: reader too) with the
//                                           game's stack and its registers
//   probe_repeat_ms = N                     repeat the peeks every N ms (0 once)
//   probe_run = N                           change to run again as they are
//
// <expr>: hex numbers (0x optional), base (the exe's load address), + and -,
// [e] for the qword at e. Example: [[0x16813D64080+1C0]]+60.
// Addresses change every launch: read them from this session's log (the hunt
// logs its hits). Nothing runs while the keys are empty.
#pragma once

namespace edvr {

class Config;

void memProbeConfigure(Config& cfg);

// Once a frame, at the render thread's frame boundary.
void memProbeFrame();

}  // namespace edvr
