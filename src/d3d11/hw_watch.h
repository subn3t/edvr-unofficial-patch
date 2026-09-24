// Hardware write-watches on game memory, for developer hunts.
//
// DR1-DR3 on every thread of the process (DR0 is pose_reader_watch's and is
// left alone): each thread is suspended for a moment while its debug
// registers are set, and threads born later are armed by the next sweep. The
// caller installs its own vectored handler, which sees EXCEPTION_SINGLE_STEP
// with DR6 bit 1..3 set for slot 0..2, and clears those bits. One user at a
// time: stereo_mode_probe.cpp's feature watch and camera_hunt.cpp's hunt.
#pragma once

#include <windows.h>

#include <cstdint>

namespace edvr {

// addr[i] 0 leaves slot i off; len[i] 1, 2, 4 or 8 bytes, the address
// aligned to it. Arms every current thread; the count armed.
int hwWatchArm(const uintptr_t addr[3], const uint8_t len[3]);

// Arms threads created since the last arm or sweep.
void hwWatchSweep();

// Every armed thread's slots off.
void hwWatchDisarm();

// The game's own frames of a stack, from a handler's context, as RVAs into
// the image [base, base + size): up to cap, innermost first.
uint32_t unwindGameStack(const CONTEXT& start, uintptr_t base, uint64_t size, uint32_t* out, uint32_t cap) noexcept;

}  // namespace edvr
