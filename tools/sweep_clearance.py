import math, random, yaml
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
from skill23_studio.nav.planning import plan_path

OCC = 50


def load_map(yaml_path):
    d = yaml.safe_load(open(yaml_path))
    res = float(d['resolution']); ox, oy = d['origin'][:2]
    pix = np.array(Image.open(Path(yaml_path).parent / d['image']).convert('L'))
    occ = np.full(pix.shape, -1, np.int16); prob = (255 - pix) / 255.
    occ[prob >= float(d['occupied_thresh'])] = 100
    occ[prob <= float(d['free_thresh'])] = 0
    return occ, res, ox, oy


def edt_for(occ, res):
    return distance_transform_edt((occ >= 0) & (occ < OCC)) * res


def opener_deg(poses):
    if len(poses) < 2:
        return 0.0
    sx, sy = poses[0][0], poses[0][1]
    gx, gy = poses[-1][0], poses[-1][1]
    fx, fy = poses[1][0] - sx, poses[1][1] - sy
    dx, dy = gx - sx, gy - sy
    if math.hypot(fx, fy) < 1e-9 or math.hypot(dx, dy) < 1e-9:
        return 0.0
    c = (fx * dx + fy * dy) / (math.hypot(fx, fy) * math.hypot(dx, dy))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


CLEARANCES = [0.08, 0.14, 0.18, 0.20, 0.24, 0.28]
N = 40

for yp in sorted(Path('/home/vmx/skill23_ws/studio_maps').glob('*/map.yaml')):
    occ, res, ox, oy = load_map(yp)
    edt = edt_for(occ, res)
    valid = (edt >= 0.20) & (occ == 0)
    lab, n = label(valid)
    if n == 0:
        print(yp.parent.name, 'no free component'); continue
    ids, cnt = np.unique(lab[lab > 0], return_counts=True)
    cid = int(ids[np.argmax(cnt)])
    ys, xs = np.where(lab == cid)
    rng = random.Random(23)
    pairs = []
    while len(pairs) < N:
        ia = rng.randrange(len(xs)); ib = rng.randrange(len(xs))
        s = (ox + (xs[ia] + .5) * res, oy + (ys[ia] + .5) * res)
        g = (ox + (xs[ib] + .5) * res, oy + (ys[ib] + .5) * res)
        if math.dist(s, g) > 1.0:
            pairs.append((s, g))
    print(f'\n== {yp.parent.name} res={res:.3f} free_cells={len(xs)}')
    for mc in CLEARANCES:
        ok = 0; openers = 0; worst = 0.0; minclear = 9.0
        for s, g in pairs:
            pl = plan_path(*s, *g, 0.0, occ, edt, ox, oy, res,
                           occupied_threshold=OCC, clearance_weight=12.0,
                           min_clearance_m=mc, los_sample_step_m=.05,
                           corner_clearance_m=.30, ridge_snap_radius_m=.10)
            if not pl:
                continue
            ok += 1
            a = opener_deg(pl)
            worst = max(worst, a)
            if a > 120.0:
                openers += 1
            for x, y, _ in pl:
                gx = int((x - ox) / res); gy = int((y - oy) / res)
                if 0 <= gy < edt.shape[0] and 0 <= gx < edt.shape[1]:
                    minclear = min(minclear, float(edt[gy, gx]))
        print(f'  clearance={mc:.2f}  plans={ok:2}/{N}  backward_openers={openers}'
              f'  worst_opener={worst:5.1f}deg  min_edt_on_path={minclear:.3f}')
