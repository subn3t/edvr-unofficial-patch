# On-foot VR: head look and stereo on foot

## Status

State (2026-09-23): flight 1 flown (166b9eb): head look works; shadows
follow the head; big head turns show culled geometry. Flight 2 built: every
exact copy of the camera rotation in the frame's constant buffers is
rotated too (lighting, shadow lookup, sky), plus b1's previous pose (3728)
and reprojection (4512). `experimental.onfoot_head_look`
(`src/d3d11/onfoot_look.cpp`); the panel itself is unchanged.

Flight 1 results:
- H1 refuted as built: only b0+64 and cb1[270..273] were rotated, and the
  deferred passes reconstruct world positions with their own copies of the
  camera rotation, so shadows and sky turned with the head ("looking up
  makes it overcast"). The flat census (elite-dangerous-dlss session 25)
  finds R or R^T, bit-exact, in b1 at 3664/3728/4432 and in 480-, 784-,
  192- and 3528-byte buffers of the passes after the G-buffer, all written
  by Map-discard.
- H2 confirmed: geometry missing on a big head turn, restored by turning
  with the stick. Milestone 2 has to widen what the game culls against.
- H3 confirmed: 30-50 main-view rewrites per frame, ~4000 other-view
  writes per second left alone; projection x 1.0524 y 1.8709 near 0.025.

Flight 2, what to look for: shadows and sky stay put in the world while the
head turns; the log's "camera copies rotated" counts are nonzero every
report and "too axis-aligned" stays near 0.

Environment measured: Quest 3 over Virtual Desktop (VirtualDesktopXR), EDVR
native OpenXR runtime, 3072x3264 per eye at 90 Hz, game build 332.841,
panel at 3840x2160 (vScreen resolution auto).

Goal, in milestones:
1. Head look (this probe): the panel scene's camera turned by the head.
2. Full view: the scene rendered at each eye's field of view and composited
   full-view into the eyes instead of onto the panel quad.
3. Stereo: alternate-eye rendering (camera offset by half the IPD on
   alternating frames, the stale eye reprojected).

Open hypotheses for flight 1 (what to look for in the headset):
- H1: lighting, shadows and the sky follow a camera rewritten only at b0+64
  and cb1[270..273]. Refuted if light or shadows stay put on screen while
  the view turns, or shade the wrong surfaces.
- H2: the game's culling (for its own, unturned camera) leaves visible gaps
  at the edges on a big head turn. Measure how far the head can turn first.
- H3: the main-view test (projection within 2%) catches the panel view and
  nothing else. Log: `onfoot look:` lines, rewrites per frame ~1-2 each for
  b0 and b1, "other views left alone" nonzero.

Next flight: `onfoot_head_look = 1`, disembark in a station, look around
slowly, then quickly; turn with the stick while looking sideways.

Ruled out: none yet.

## 2026-09-23: what the game renders on foot (census)

Three one-frame censuses on foot (dump_draws, census_offscreen = 1), a
short walk from a settlement, ship dismissed:

- 2 eye draws per frame: VS 5C36AF051B98B9F1, 6 indices, the panel
  (3840x2160, SRV0) pasted into each 3072x3148 eye target.
- ~1400 offscreen draws, 42 dispatches, 14 copies: the whole scene, once,
  mono. Main G-buffer 3840x2160 (R10G10B10A2 + D32S8), shadow atlas
  6144x4096 in six 2048 tiles, a 256x256 second view drawn with the
  G-buffer shaders, bloom chain from 1920x1080, and two unidentified
  passes at 4608x3968 and 4016x2412.
- The G-buffer object shaders read the flat game's registers: cb1[270..273]
  clip transform by columns, cb1[275] origin, t33 object pool (stride 336);
  the simple families read b0's A at cb0[4..7].
- census_cb_watch dumps at most 768 bytes (kCbShadowBytes), so it cannot
  show cb1[270+].
- Head tracking does nothing to the panel's content (field, 2026-09-23).

## 2026-09-23 (later): shadows follow head pitch

Field: neutral head right; head pitched down, no shadows (the ship's
shadow is gone); head up, everything shadowed; looking with the stick is
fine. Ruled out, each by a flight with no visible change:
- ruled out: rotating whole clip-transform, R and R^T copies (to 1e-3), lone
  camera axes, b1's previous pose (3728) and reprojection (4512), because
  `onfoot_head_look_skip` leaving all of them out changes nothing.
- ruled out: the 448-byte "cascade" planes as the lookup: four on-foot
  captures show them world-fixed in camera space, but only compute shaders
  read them (ADE0E10C2FD288AA first dispatch; 5998146D464F5C0E and
  EB0245DE0BB23BB6 indirect) -- caster culling, not lookup.
- ruled out: a camera-space sun, or view-to-light / screen-to-light matrix,
  in any Map-discard buffer up to 16 KB (captures, any length).
Open: `outside` (build 496e49d) turns the view only while a panel-sized
target is bound, for the hypothesis that the shadow-map and other offscreen
passes cull casters with the main camera.
