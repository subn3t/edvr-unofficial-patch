# The on-foot frame: a field catalogue

What Elite Dangerous (Odyssey, game build 332.841) renders on foot, and where it
keeps the camera, as found by the head-look work (`src/d3d11/onfoot_look.cpp`,
docs/onfoot-vr.md). Every entry names how it was found so it can be checked
again after a game update. Offsets are bytes; "row n" is the float4 at 16*n.

## Method

- **Census** (`hotkey.dump_draws`, `census_offscreen = 1`): every draw of one
  frame with its targets, SRVs (`s=` t0-3, `x=` t4-7), VS cb0 (`c=`) and
  shader hashes. Each census has its own `DC id @n` table mapping ids to
  resources. Pixel-stage constant buffers are not in the census.
- **Shaders**: the census dumps bytecode to `edvr_logs\shaders\*.dxbc`;
  disassemble with Windows Kits `fxc.exe /dumpbin /Fc out.asm in.dxbc`. The
  `dcl_constantbuffer CBn[rows]` line gives the size read; `cbN[r]` uses
  give the rows.
- **Capture** (the census key with `onfoot_head_look = 1`):
  `edvr_logs\onfoot_cb_<n>.bin`, every b0/b1 and Map-discard CB/SRV buffer
  write of 48 B-16 KB, plus whole-buffer `UpdateSubresource` writes, as the
  game wrote them, then the camera R, head Q and previous Q. Captures taken
  facing different ways separate camera-relative data from world data:
  - world-fixed in camera space: `R^T v / |v|` constant across captures;
  - camera-fixed in world space: `R v / |v|` constant;
  - compare directions (scale-free), both contiguous and stride-4 (columns).
- **Field A/B**: `experimental.onfoot_head_look_skip` leaves parts out live.

Camera convention: R = rows right, up, forward (world to view), from b1's
clip transform at 4320. The view is +z forward, +y up, reversed-Z.

## Views and projections

Per-view b0 (208 bytes, clip rows at +64) and b1 (5376 bytes) are rewritten
for each view in the frame:

| View | Projection (x, y, near) | Notes |
|---|---|---|
| Main | 1.0524, 1.8709, 0.025 (aspect 0.5625) | also seen as 0.562, 1.0 in another session |
| Atmosphere sphere | main x/y scaled by planet radius (e.g. 2954720, 5252836), near 0.025 | model scale S folded into the clip, w row = forward * S |
| Helmet HUD | 1.294, 2.3, near 0.0675 | head-locked; must not turn |
| Unknown | 1.294, 2.3, near 0.1; 1.052, 1.871, near 0.1 | a few draws late in the frame |
| 256x256 view | 0.935, 0.887 | drawn with the G-buffer shaders; probe/reflection? |
| Shadow cascades | orthographic (w row 0,0,0,1) | six 2048 tiles of a 6144x4096 atlas |

## b1 (5376 bytes, the frame buffer)

| Offset | Row | Contents | Found by |
|---|---|---|---|
| 48 | 3 | camera position, planet-local, unit, w 1 (world) | atmosphere PS 48BC9A87 |
| 64 | 4 | sun direction (world) | same |
| 400 | 25 | minus planet up (world) | same |
| 496 | 31 | .z/.w atmosphere inner/outer radius (5150, 5252.8) | same |
| 656 | 41-44 | clip transform copy | flat census |
| 3664 | 229-231 | R^T | capture |
| 3728 | 233-235 | previous frame [R_prev^T, pos] | capture |
| 3872 | 242 | forward axis | capture |
| 4320 | 270-273 | clip transform by columns (object shaders read cb1[270..273]) | flat census |
| 4384 | 274 | forward axis | capture |
| 4400 | 275 | camera origin | flat census |
| 4432 | 277-279 | R | capture |
| 4512 | 282-284 | reprojection [D, D dpos] | flat census |
| 5248 | 328-330 | screen-to-world rays by columns: 0.95 right, 0.534 up, (other), forward | capture |

Rows that vary with the camera but are not rigid in either space: 35, 36, 38,
118 (near camera-up, lagged?), 310-315 (six similar vectors). Scalars that
vary: 61, 62, 84, 136, 210, 226 (time-like).

## Other constant buffers

| Size | Where | Contents |
|---|---|---|
| 192 | several passes | R^T at 128 |
| 208 | b0 | per view/draw: clip rows at +64; object matrix rows 9-11 |
| 224 | atmosphere sphere cb2 (PS F9FA59D2 / 35C896D1) | rows 0-3 P V S (planet scale); rows 8-10 planet scale+centre; row 12 planet up |
| 288 | many (terrain tiles) | first write of the frame: row 1 the planet centre in camera space |
| 448 | compute culling (CS ADE0E10C, 5998146D, EB0245DE) | four 112-byte shadow cascade records, plane normals in camera space |
| 48 | before the 448 | row 1 the shadow light direction in camera space; row 2 (1,0,0,0) |
| 480 | deferred passes | R at 32 and 320, [R \| -R c] at 96/256, screen-to-world rays at 192 |
| 592 | sun shadow mask PS 7EAC7196 | row 0 target size; rows 1-4 inverse projection; rows 7-9 view-to-light R L^T; rows 11-34 per-cascade scale/offset; row 35 cascade count |
| 784 | deferred sun lighting PS 7CECABDE | R^T at 32, [R \| -R c] at 96, rays at 224 (VS reads rows 14-16), light dir row 42, sun colour row 40 |
| 944 | terrain G-buffer (VS 72BDD292, PS F1670378 / 9642F473) | row 2 planet centre in camera space, row 10 planet rotation quaternion (camera-relative; consistent with the unturned object matrix in b0 rows 9-11, so left alone) |

## Passes, in order (on foot, one frame)

1. Shadow atlas 6144x4096 (six 2048 tiles), orthographic views.
2. Compute: cascade culling (448-byte records).
3. G-buffer 3840x2160 (R10G10B10A2 + D32S8); terrain patches, objects.
4. Atlas prefilter: CS DAB798FB (atlas to 3072x2048 R16G16B16A16), then
   CS F2585F30 / E86805B8 blur passes.
5. Sun shadow mask: full-screen PS 7EAC7196 (+ B403F48C) into an R8 target at
   panel size; depth, G-buffer normals, the filtered atlas.
6. Deferred sun lighting: PS 7CECABDE (784-byte cb2).
7. Star skybox: VS F8FA801F (inverse view-projection in 240-byte cb2 rows
   11-14), PS 84965D3C (two cube maps).
8. Sun sprite (PS A121912A, 512x4 LUT), atmosphere sphere (15360 indices;
   PS 48BC9A87 in full and at 480x270, F9FA59D2, 35C896D1).
9. Forward-lit transparent (PS 3B0B38CD, AFED1D4B; many b1 rows).
10. Colour grading (PS 68ABCB9F), bloom chain from 1920x1080 down to 120x67,
    tonemap, then the panel composite into each eye (VS 5C36AF05).

## Open questions

- The 256x256 view: what it is and whether it must turn.
- The two passes at 4608x3968 and 4016x2412.
- b1 rows 35-38, 118 and 310-315.
