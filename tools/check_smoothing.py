"""Measure smoothness, planner-corridor deviation and clearance on real maps."""
import math, random, yaml
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


def edt_for(occ, res):
    return distance_transform_edt((occ >= 0) & (occ < OCC)) * res


def dist_to_poly(p, poly):
    best = 1e9
    for a, b in zip(poly, poly[1:]):
        dx = b[0] - a[0]; dy = b[1] - a[1]; q = dx * dx + dy * dy
        t = 0.0 if q < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / q))
        best = min(best, math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy)))
    return best


def geom(path):
    pts = [(p[0], p[1]) for p in path]
    yaws, gaps = [], []
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        gaps.append(L)
        if L > 1e-6:
            yaws.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    steps = [abs(math.atan2(math.sin(b - a), math.cos(b - a))) for a, b in zip(yaws, yaws[1:])]
    sharp = sum(1 for s in steps if s > math.radians(20))
    return dict(
        maxstep=math.degrees(max(steps, default=0.0)),
        meanstep=math.degrees(sum(steps) / len(steps)) if steps else 0.0,
        sharp=sharp,
        total=math.degrees(sum(steps)),
        maxgap=max(gaps, default=0.0),
    )


for yp in sorted(Path('/home/vmx/skill23_ws/studio_maps').glob('*/map.yaml')):
    occ, res, ox, oy = load_map(yp)
    edt = edt_for(occ, res)
    valid = (edt >= 0.20) & (occ == 0)
    lab, n = label(valid)
    ids, cnt = np.unique(lab[lab > 0], return_counts=True)
    ys, xs = np.where(lab == int(ids[np.argmax(cnt)]))
    rng = random.Random(23)
    agg = {'pl': [], 'sm': [], 'dev': [], 'clear': [], 'gap': []}
    for _ in range(10):
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
        if not pl:
            continue
        sm = smooth_path_poses(pl, edt, ox, oy, res, occupancy=occ,
                               occupied_thresh=OCC, min_clearance_m=CLEAR)
        poly = [(p[0], p[1]) for p in pl]
        agg['pl'].append(geom(pl)); agg['sm'].append(geom(sm))
        agg['dev'].append(max(dist_to_poly((p[0], p[1]), poly) for p in sm))
        mc = 9.0
        for x, y, _ in sm:
            gx = int((x - ox) / res); gy = int((y - oy) / res)
            if 0 <= gy < edt.shape[0] and 0 <= gx < edt.shape[1]:
                mc = min(mc, float(edt[gy, gx]))
        agg['clear'].append(mc)
        agg['gap'].append(geom(sm)['maxgap'])
    if not agg['pl']:
        print(yp.parent.name, 'no plans'); continue
    def avg(rows, k):
        return sum(r[k] for r in rows) / len(rows)
    print(f'\n== {yp.parent.name} res={res:.3f}  ({len(agg["pl"])} paths)')
    print(f'  planned : max yaw step {avg(agg["pl"], "maxstep"):5.1f}deg  '
          f'mean {avg(agg["pl"], "meanstep"):4.1f}deg  '
          f'sharp(>20deg) {avg(agg["pl"], "sharp"):4.1f}  '
          f'total turning {avg(agg["pl"], "total"):6.0f}deg')
    print(f'  smoothed: max yaw step {avg(agg["sm"], "maxstep"):5.1f}deg  '
          f'mean {avg(agg["sm"], "meanstep"):4.1f}deg  '
          f'sharp(>20deg) {avg(agg["sm"], "sharp"):4.1f}  '
          f'total turning {avg(agg["sm"], "total"):6.0f}deg')
    print(f'  corridor: max dev from planner {max(agg["dev"]):.3f} m '
          f'(mean {sum(agg["dev"]) / len(agg["dev"]):.3f})')
    print(f'  safety  : min clearance {min(agg["clear"]):.3f} m  '
          f'max point gap {max(agg["gap"]):.3f} m')
