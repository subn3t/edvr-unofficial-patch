// On foot: the head turns the game's own camera (experimental.onfoot_head_look).
//
// WHY THIS EXISTS
//
// On foot, Elite renders the scene once, mono, into the flat panel
// (docs/onfoot-vr.md: 2026-09-23 census, ~1400 offscreen draws and exactly
// two eye draws, the panel composite). Head tracking does nothing to what the
// panel shows. The panel scene's vertex shaders read the camera exactly as
// the flat game does: the per-view b0 carries the clip transform's rows at
// offset 64 (A), and the object-table families read it again by columns at
// b1 offset 4320 (cb1[270..273]).
//
// This turns those two for the main view: at the Unmap of each write, before
// the bytes reach the GPU, the head's rotation (headPose, the runtime's own
// pose, before any EDVR offset) is composed into the view -- clip =
// P * Q^T * V, with Q the head rotation in the game's view axes. The panel is
// left where it is; what it shows now turns with the head.
//
// The passes after the G-buffer carry their own copies of the camera, and
// each one left alone shows as a fault that follows the head
// (docs/cobra-onfoot-frame.md): the same camera through another projection
// (the atmosphere sphere), its clip transform or inverse in another world
// frame (the star skybox, in the galaxy's), a scaled clip transform (the
// atmosphere's P V S), and the sun's shadow mask's view-to-light rotation.
// Those are turned too, found by what they are, not by where they sit. The
// game's own culling still uses the unturned camera, so a big head turn shows
// what it did not draw at the edges.
//
// WHICH WRITES ARE THE MAIN VIEW
//
// The per-view buffers are rewritten for every view in a frame (shadow
// cascades, the 256x256 view, the main one). The first G-buffer draw into a
// panel-sized target names the buffers and records the main projection's x/y
// scale and near plane; a later write is the main view when its projection
// matches that within 2% (the flat mod's rule, elite-dangerous-dlss
// src/dlss/camera.cpp). Orthographic cascades and other cameras are far off.
//
// Off by default. Off, every entry point below is one bool load.
#pragma once

#include <d3d11.h>

#include <cstdint>

namespace edvr {

class Config;

void onFootLookConfigure(Config& cfg);

namespace detail {
extern bool g_onFootLookEnabled;
}  // namespace detail
inline bool onFootLookEnabled() { return detail::g_onFootLookEnabled; }

// At every game draw on the owner context (after vscreen's verdict, which
// leaves the draw to the game). Learns the buffers and the main projection at
// the first panel-sized G-buffer draw of a frame; returns at once after it.
void onFootLookBeforeDraw();

// Map of any resource on the owner context: remembers the pointer of the
// learned per-view buffers and of other small buffers rewritten whole.
void onFootLookMapped(ID3D11Resource* res, void* data, D3D11_MAP type);

// Unmap, BEFORE the real one (after it the memory is no longer ours): turns
// the camera in the write, and keeps a shadow of b0's clip rows.
void onFootLookBeforeUnmap(ID3D11Resource* res);

// ClearState or a command list with restore off: every binding is gone.
void onFootLookStateCleared();

void onFootLookFrameBoundary();

}  // namespace edvr
