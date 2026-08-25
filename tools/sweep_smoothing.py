"""Sweep smoother relaxation settings: smoothness vs corridor deviation."""
import math, random, time, yaml
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, label
from skill23_studio.nav.planning import plan_path
from skill23_studio.nav.smoothing import smooth_path_poses

OCC = 50
CLEAR = 0.18


def load_map(yp):
    d = yaml.safe_load(open(yp))
    res = float(d['resolution']); ox, oy = d['origin'][:2]
    pix = np.array(Image.open(Path(yp).parent / d['image']).convert('L'))
    occ = np.full(pix.shape, -1, np.int16); prob = (255 - pix) / 255.
    occ[prob >= float(d['occupied_thresh'])] = 100
    occ[prob <= float(d['free_thresh'])] = 0
    return occ, res, ox, oy


def dist_to_poly(p, poly):
    best = 1e9
    for a, b in zip(poly, poly[1:]):
        dx = b[0] - a[0]; dy = b[1] - a[1]; q = dx * dx + dy * dy
        t = 0.0 if q < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / q))
        best = min(best, math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy)))
    return best


def steps_deg(path):
    pts = [(p[0], p[1]) for p in path]
    yaws = []
    for a, b in zip(pts, pts[1:]):
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
            yaws.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return [math.degrees(abs(math.atan2(math.sin(b - a), math.cos(b - a))))
            for a, b in zip(yaws, yaws[1:])]


cases = []
for yp in sorted(Path('/home/vmx/skill23_ws/studio_maps').glob('*/map.yaml')):
    occ, res, ox, oy = load_map(yp)
    edt = distance_transform_edt((occ >= 0) & (occ < OCC)) * res
    valid = (edt >= 0.20) & (occ == 0)
    lab, n = label(valid)
    ids, cnt = np.unique(lab[lab > 0], return_counts=True)
    ys, xs = np.where(lab == int(ids[np.argmax(cnt)]))
    rng = random.Random(23)
    for _ in range(6):
        while True:
            ia = rng.randrange(len(xs)); ib = rng.randrange(len(xs))
            s = (ox + (xs[ia] + .5) * res, oy + (ys[ia] + .5) * res)
            g = (ox + (xs[ib] + .5) * res, oy + (ys[ib] + .5) * res)
            if math.dist(s, g) > 1.5:
                break
        pl = plan_path(*s, *g, 0.0, occ, edt, ox, oy, res, occupied_threshold=OCC,
                       clearance_weight=12.0, min_clearance_m=CLEAR,
                       los_sample_step_m=.05, corner_clearance_m=.30,
                       ridge_snap_radius_m=.10)
        if pl:
            cases.append((pl, occ, edt, ox, oy, res))

print(f'{len(cases)} planned paths\n')
print(f'{"iters":>5} {"weight":>6} {"fillet":>6} | {"max step":>8} {"mean":>5} '
      f'{"sharp":>5} | {"max dev":>7} | {"min clr":>7} | {"ms/path":>7}')
for iters, weight, fillet in (
    (60, 0.40, 0.18), (80, 0.35, 0.18), (80, 0.40, 0.18),
    (100, 0.35, 0.18), (100, 0.40, 0.18),
):
    mx = 0.0; means = []; sharp = 0; dev = 0.0; clr = 9.0; t0 = time.time()
    for pl, occ, edt, ox, oy, res in cases:
        sm = smooth_path_poses(pl, edt, ox, oy, res, occupancy=occ,
                               occupied_thresh=OCC, min_clearance_m=CLEAR,
                               fillet_radius_m=fillet,
                               relax_iterations=iters, relax_weight=weight)
        st = steps_deg(sm)
        mx = max(mx, max(st, default=0.0))
        means.append(sum(st) / len(st) if st else 0.0)
        sharp += sum(1 for s in st if s > 20.0)
        poly = [(p[0], p[1]) for p in pl]
        dev = max(dev, max(dist_to_poly((p[0], p[1]), poly) for p in sm))
        for x, y, _ in sm:
            gx = int((x - ox) / res); gy = int((y - oy) / res)
            if 0 <= gy < edt.shape[0] and 0 <= gx < edt.shape[1]:
                clr = min(clr, float(edt[gy, gx]))
    ms = (time.time() - t0) / len(cases) * 1000.0
    print(f'{iters:5} {weight:6.2f} {fillet:6.2f} | {mx:8.1f} '
          f'{sum(means) / len(means):5.1f} {sharp:5} | {dev:7.3f} | {clr:7.3f} | {ms:7.1f}')
