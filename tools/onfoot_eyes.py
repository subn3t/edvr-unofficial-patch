"""Where do the two eyes part? Reads an on-foot eye dump (edvr_logs/onfoot_eyes_<n>,
written with the census-key capture while the on-foot stereo runs) and pairs each
texture the first pipeline made with the second's twin -- the same shaders wrote
them, at the same size and format. For each pair it lines the eyes up by depth
(the eyes differ only by their offset along the camera's right axis, so a pixel
of one eye at depth D sits sx * (o_other - o_this) / D * width/2 pixels away in
the other) and reports how differently the two show the same surfaces, overall
and in the dark (the shade the eyes lit differently, 2026-09-25). The stages come
in draw order: the first pair whose dark areas differ is where the eyes part.

    python tools/onfoot_eyes.py <dump dir> [--png]

--png also writes, per pair, left | right lined up with the left | difference,
tone-mapped alike, a quarter size, to <dump dir>/report/.
"""
import math
import os
import sys

import numpy as np


def f11(v):  # 11-bit float: 5 exponent, 6 mantissa
    e = (v >> 6) & 31
    m = v & 63
    return np.where(e == 0, m / 64.0 * 2.0 ** -14, 2.0 ** (e.astype(np.float64) - 15) * (1 + m / 64.0))


def f10(v):  # 10-bit float: 5 exponent, 5 mantissa
    e = (v >> 5) & 31
    m = v & 31
    return np.where(e == 0, m / 32.0 * 2.0 ** -14, 2.0 ** (e.astype(np.float64) - 15) * (1 + m / 32.0))


def decode(raw, w, h, fmt):
    """The texture as float32 (h, w, channels); None for a format not read."""
    if fmt in (1, 2):
        return np.frombuffer(raw, np.float32).reshape(h, w, 4)
    if fmt in (9, 10):
        return np.frombuffer(raw, np.float16).reshape(h, w, 4).astype(np.float32)
    if fmt == 11:
        return np.frombuffer(raw, np.uint16).reshape(h, w, 4) / 65535.0
    if fmt == 26:
        u = np.frombuffer(raw, np.uint32).reshape(h, w)
        return np.stack([f11(u & 0x7FF), f11((u >> 11) & 0x7FF), f10((u >> 22) & 0x3FF)], -1).astype(np.float32)
    if fmt in (23, 24, 25):
        u = np.frombuffer(raw, np.uint32).reshape(h, w)
        return np.stack([(u & 1023) / 1023.0, ((u >> 10) & 1023) / 1023.0, ((u >> 20) & 1023) / 1023.0,
                         (u >> 30) / 3.0], -1).astype(np.float32)
    if fmt in (27, 28, 29, 30):
        return np.frombuffer(raw, np.uint8).reshape(h, w, 4) / 255.0
    if fmt in (87, 88, 90, 91):
        return (np.frombuffer(raw, np.uint8).reshape(h, w, 4)[..., [2, 1, 0, 3]]) / 255.0
    if fmt in (33, 34):
        return np.frombuffer(raw, np.float16).reshape(h, w, 2).astype(np.float32)
    if fmt == 35:
        return np.frombuffer(raw, np.uint16).reshape(h, w, 2) / 65535.0
    if fmt in (15, 16):
        return np.frombuffer(raw, np.float32).reshape(h, w, 2)
    if fmt in (39, 40, 41):
        return np.frombuffer(raw, np.float32).reshape(h, w, 1)
    if fmt in (19, 20, 21):
        return np.frombuffer(raw, np.float32).reshape(h, w, 2)[..., :1]
    if fmt in (44, 45, 46):
        return ((np.frombuffer(raw, np.uint32).reshape(h, w, 1) & 0xFFFFFF) / 16777215.0).astype(np.float32)
    if fmt in (53, 54):
        return np.frombuffer(raw, np.float16).reshape(h, w, 1).astype(np.float32)
    if fmt in (55, 56, 57):
        return np.frombuffer(raw, np.uint16).reshape(h, w, 1) / 65535.0
    if fmt in (48, 49, 50):
        return np.frombuffer(raw, np.uint8).reshape(h, w, 2) / 255.0
    if fmt in (60, 61, 62, 65):
        return np.frombuffer(raw, np.uint8).reshape(h, w, 1) / 255.0
    if fmt == 67:
        u = np.frombuffer(raw, np.uint32).reshape(h, w)
        e = 2.0 ** ((u >> 27).astype(np.float64) - 24)
        return np.stack([(u & 511) * e, ((u >> 9) & 511) * e, ((u >> 18) & 511) * e], -1).astype(np.float32)
    return None


DEPTH_FORMATS = {19, 20, 21, 39, 40, 44, 45, 46, 55}


def read_index(d):
    meta, texs = {}, []
    for line in open(os.path.join(d, 'index.txt'), encoding='utf-8'):
        parts = line.split('|')
        head = parts[0].split()
        if head and head[0] == 'stereo':
            it = iter(head)
            for k in it:
                meta[k] = next(it)
            continue
        if not head or head[0] != 'tex':
            continue
        w, h = map(int, head[2].split('x'))
        t = dict(id=int(head[1]), w=w, h=h, fmt=int(head[4]), bind=int(head[6], 16), writes=int(head[10]),
                 reads=int(head[12]), status=head[13], uses=[])
        for p in parts[1:]:
            f = p.split()
            vw, vh = map(int, f[3].split('x'))
            t['uses'].append(dict(ord=int(f[0]), pipe=int(f[1]), slot=int(f[2]), vw=vw, vh=vh, vs=f[4], ps=f[5]))
        t['wuses'] = [u for u in t['uses'] if u['slot'] < 16]
        t['ruses'] = [u for u in t['uses'] if u['slot'] >= 16]
        t['pipes'] = sorted({u['pipe'] for u in (t['wuses'] or t['ruses'])})
        t['first'] = (t['wuses'] or t['ruses'])[0]['ord'] if t['uses'] else 1 << 30
        texs.append(t)
    return meta, texs


def load(d, t):
    path = os.path.join(d, 't%d.raw' % t['id'])
    if t['status'] != 'saved' or not os.path.exists(path):
        return None
    return decode(open(path, 'rb').read(), t['w'], t['h'], t['fmt'])


def lum(a):
    if a.shape[2] >= 3:
        return 0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]
    return a[..., 0]


def main():
    d = sys.argv[1]
    png = '--png' in sys.argv
    meta, texs = read_index(d)
    left = int(meta.get('left_pipe', 0))
    half = float(meta.get('half_ipd_m', 0))
    anchor = int(meta.get('anchor', 0))
    sx = float(meta.get('sx', 1))
    near = float(meta.get('near', 0.025))
    pw, ph = map(int, meta.get('panel', '0x0').split('x'))
    offs = {}
    for p in (0, 1):
        base = -half if p == left else half
        offs[p] = base - anchor * half
    print('meta:', meta, ' eye offsets (m):', offs)

    # Each pipeline's scene depth: the panel-sized depth it wrote most.
    depth = {}
    for p in (0, 1):
        cands = [t for t in texs if t['fmt'] in DEPTH_FORMATS and any(u['slot'] == 8 and u['pipe'] == p for u in t['wuses'])]
        cands.sort(key=lambda t: -t['writes'])
        for t in cands:
            a = load(d, t)
            if a is not None:
                vw, vh = t['wuses'][0]['vw'] or t['w'], t['wuses'][0]['vh'] or t['h']
                depth[p] = (t['id'], a[:vh, :vw, 0])
                break
    print('scene depth:', {p: v[0] for p, v in depth.items()})

    # Pairs: the same shaders wrote them (or first read them), same size and format.
    groups = {}
    for t in texs:
        u = (t['wuses'] or t['ruses'] or [None])[0]
        if u is None:
            continue
        key = (t['w'], t['h'], t['fmt'], u['vs'], u['ps'], u['slot'] % 16 if u['slot'] < 16 else u['slot'])
        groups.setdefault(key, []).append(t)
    pairs, shared = [], []
    for key, g in groups.items():
        if len(g) == 2:
            g.sort(key=lambda t: t['first'])
            a, b = g
            pa = a['pipes'][0] if len(a['pipes']) == 1 else None
            pb = b['pipes'][0] if len(b['pipes']) == 1 else None
            if pa in (0, 1) and pb in (0, 1) and pa != pb:
                l, r, how = (a, b, 'by pipeline') if pa == left else (b, a, 'by pipeline')
            else:
                l, r, how = (a, b, 'assumed: first made = first pipeline') if left == 0 else (b, a, 'assumed')
            pairs.append((l, r, how))
    for t in texs:
        rp = {u['pipe'] for u in t['ruses']}
        if 0 in rp and 1 in rp:
            shared.append(t)

    out = os.path.join(d, 'report')
    if png:
        os.makedirs(out, exist_ok=True)
    rows = []
    for l, r, how in sorted(pairs, key=lambda p: p[0]['first']):
        A, B = load(d, l), load(d, r)
        if A is None or B is None:
            continue
        u = l['wuses'][0] if l['wuses'] else l['ruses'][0]
        vw = min(u['vw'] or l['w'], l['w'])
        vh = min(u['vh'] or l['h'], l['h'])
        A, B = A[:vh, :vw], B[:vh, :vw]
        LA, LB = lum(A), lum(B)
        note = ''
        # Line the right eye's pixels up with the left's by the right eye's depth.
        warped = None
        rpipe, lpipe = (1 - left), left
        if rpipe in depth and l['fmt'] not in DEPTH_FORMATS and pw and vw > 16:
            z = depth[rpipe][1]
            scale_x, scale_y = z.shape[1] / vw, z.shape[0] / vh
            yi = np.minimum((np.arange(vh) * scale_y).astype(int), z.shape[0] - 1)
            xi = np.minimum((np.arange(vw) * scale_x).astype(int), z.shape[1] - 1)
            zr = z[yi][:, xi]
            dist = np.where(zr > 0, near / np.maximum(zr, 1e-12), np.inf)
            shift = sx * (offs[rpipe] - offs[lpipe]) / dist * (vw / 2.0)  # left x = right x + shift
            xs = np.arange(vw)[None, :] + shift
            xl = np.rint(xs).astype(np.int64)
            ok = (xl >= 0) & (xl < vw)
            if lpipe in depth:
                zl = depth[lpipe][1][yi][:, xi]
                zl_at = np.take_along_axis(zl, np.clip(xl, 0, vw - 1), 1)
                dl = np.where(zl_at > 0, near / np.maximum(zl_at, 1e-12), np.inf)
                ok &= np.where(np.isfinite(dist), np.abs(dl - dist) < 0.03 * dist, ~np.isfinite(dl))
            la_at = np.take_along_axis(LA, np.clip(xl, 0, vw - 1), 1)
            warped = (la_at, ok)
        if warped is not None:
            la_at, ok = warped
            lr, ll = LB[ok], la_at[ok]
            finite = np.isfinite(lr) & np.isfinite(ll)
            lr, ll = lr[finite], ll[finite]
            if lr.size > 1000:
                dark = lr <= np.percentile(lr, 25)
                mean = max(float(np.mean(np.abs(lr))), 1e-9)
                diff = float(np.mean(np.abs(ll - lr))) / mean
                dr = float(np.mean(lr[dark])) if dark.any() else 0.0
                dl = float(np.mean(ll[dark])) if dark.any() else 0.0
                ratio = dl / dr if dr > 1e-9 else float('nan')
                note = 'lined up %4.1f%% of pixels; mean |L-R| %.3f of mean; dark quarter L/R %.3f (%.4g / %.4g)' % (
                    100.0 * ok.mean(), diff, ratio, dl, dr)
        if not note:
            ma, mb = float(np.nanmean(LA)), float(np.nanmean(LB))
            note = 'not lined up; means L %.4g R %.4g (L/R %.3f)' % (ma, mb, ma / mb if mb else float('nan'))
        rows.append((u['ord'], l['id'], r['id'], l['w'], l['h'], l['fmt'], u['vs'], u['ps'], u['slot'], how, note))
        if png:
            from PIL import Image
            ref = float(np.nanpercentile(np.abs(LB), 50)) or 1.0
            def tm(x):
                x = np.nan_to_num(np.abs(x) / (ref * 4), nan=0, posinf=1)
                return (255 * (x / (1 + x)) ** (1 / 2.2)).clip(0, 255).astype(np.uint8)
            panels = [tm(LA), tm(LB)]
            if warped is not None:
                la_at, ok = warped
                dimg = np.where(ok, np.abs(la_at - LB), 0)
                panels.append(tm(dimg * 4))
            img = np.concatenate([p[::4, ::4] for p in panels], 1)
            Image.fromarray(img).save(os.path.join(out, 'o%05d_t%d_t%d.png' % (u['ord'], l['id'], r['id'])))

    print('\npairs (left, right), in draw order -- the first whose dark quarter differs is where the eyes part:')
    for r in rows:
        print('ord %5d  t%-4d t%-4d %5dx%-5d fmt %2d vs %s ps %s slot %2d  [%s]\n        %s' % r)
    print('\ntextures both pipelines read (one texture, both eyes):')
    for t in sorted(shared, key=lambda t: t['first']):
        wp = sorted({u['pipe'] for u in t['wuses']})
        print('  t%-4d %5dx%-5d fmt %2d bind 0x%X written by pipes %s (%d writes), read by both (%d reads); first %d' % (
            t['id'], t['w'], t['h'], t['fmt'], t['bind'], wp or 'none seen (compute or earlier)', t['writes'],
            t['reads'], t['first']))


if __name__ == '__main__':
    main()
