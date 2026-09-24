// The engine's own stereo render mode: a probe (experimental.stereo_mode_probe).
//
// WHY THIS EXISTS
//
// On foot Elite draws one mono panel and pastes it into the eyes; in the
// cockpit it renders true stereo. The engine has a data-driven switch between
// such modes: a game-logic activity, SetIdentStereoRenderModeActivity, whose
// Execute (build 332841, RVA 0x283AFB0) reads a mode from its parameters,
// clamps it to 0..2 (anything else becomes 1), and passes it to a virtual
// setter (vtable offset 0x148) on an object it looks up from a service:
//
//     tgt = lookup(owner, typeId @0x145F02BB4)->vfunc[0xA0]()
//     mode = params->[+8]; tgt->vfunc[0x148](mode < 3 ? mode : 1)
//
// Found by string reference and disassembly (docs/onfoot-vr.md, "Engine
// stereo mode"). What the three modes mean, and whether the activity runs at
// all when you step out of the ship, is what this measures: every call is
// logged with the mode, the caller and whether the journal says on foot.
// experimental.stereo_mode_override (0..2; -1 off) substitutes a mode for the
// one call and puts the parameter back after it -- the experiment.
//
// Hooks the game's code (common/code_hook.h, the relay of
// object_record_writer_hook.cpp): refuses unless the executable is build
// 332841 and the function's first bytes are the ones read from it. Off by
// default; installed once, at the first configure with it on.
#pragma once

namespace edvr {

class Config;

void stereoModeProbeConfigure(Config& cfg);

// Once a frame (vScreen's frame boundary): the ship/foot differ, which
// snapshots the stereo mode's objects after each change of the journal's
// on-foot state and logs what differs.
void stereoModeProbeFrame();

}  // namespace edvr
