"""Regression harness for the Studio planner / smoother / controller.

Checks, using the shipped defaults:
  * determinism, collision safety, clearance, point spacing
  * heading step per point (how sharply the controller is asked to turn)
  * controller-followability on the smoothed path
  * final-yaw endgame settling at the /amcl_pose rate seen in nav.log
"""
import math
import random
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, label

from skill23_studio.nav.planning import plan_path
from skill23_studio.nav.smoothing import smooth_path_poses

OCC = 50
CLEAR = 0.18
MAPS = Path('/home/vmx/skill23_ws/studio_maps')


def edt_for(occ, res):
    return distance_transform_edt((occ >= 0) & (occ < OCC)) * res


def plan(occ, edt, res, s, g, ox=0.0, oy=0.0):
    return plan_path(*s, *g, 0.0, occ, edt, ox, oy, res, occupied_threshold=OCC,
                     clearance_weight=12.0, min_clearance_m=CLEAR,
                     los_sample_step_m=.05, corner_clearance_m=.30,
                     ridge_snap_radius_m=.10)


def smooth(pl, edt, res, occ, ox=0.0, oy=0.0):
    return smooth_path_poses(pl, edt, ox, oy, res, occupancy=occ,
                             occupied_thresh=OCC, min_clearance_m=CLEAR)


def sample_poly(path, step=0.01):
    out = []
    for a, b in zip(path, path[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, math.ceil(L / step))
        out += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
                for i in range(n)]
    if path:
        out.append((path[-1][0], path[-1][1]))
    return out


def at(arr, x, y, ox, oy, res):
    gx = int((x - ox) / res); gy = int((y - oy) / res)
    if 0 <= gy < arr.shape[0] and 0 <= gx < arr.shape[1]:
        return float(arr[gy, gx])
    return 0.0


def metrics(sm, occ, edt, ox, oy, res):
    pts = sample_poly(sm)
    clear = min((at(edt, x, y, ox, oy, res) for x, y in pts), default=0.0)
    bad = sum(1 for x, y in pts
              if at(occ, x, y, ox, oy, res) >= OCC or at(occ, x, y, ox, oy, res) < 0)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(sm, sm[1:])]
    yaws = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(sm, sm[1:])
            if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6]
    steps = [abs(math.atan2(math.sin(b - a), math.cos(b - a)))
             for a, b in zip(yaws, yaws[1:])]
    return dict(clear=clear, bad=bad, maxgap=max(gaps, default=0.0),
                maxstep=math.degrees(max(steps, default=0.0)))


def controller_sim(path, v=0.30, w_align=0.30, w_fine=0.16, min_align=0.16,
                   min_steer=0.05, lookahead=0.10, align=0.25, aligned=0.15,
                   dead=0.04, dt=0.05, sensor_hz=3.5, limit=120.0):
    """Unicycle sim of cn_controller: stale pose between /amcl_pose updates.

    Angular commands are duty; nav.log shows 0.30 duty turning ~1.0 rad/s, so
    duty is scaled by that measured gain.
    """
    gain = 3.33
    pts = [(p[0], p[1]) for p in path]
    x, y = pts[0]
    yaw = path[0][2]
    sx, sy, syaw = x, y, yaw
    idx = 0
    t = 0.0
    next_sensor = 0.0
    cmd = (0.0, 0.0)
    max_ct = 0.0
    reversals = 0
    last_sign = 0
    state = 'ALIGN'
    rot_dir = 0.0

    def closest(px, py, start):
        best = (1e9, pts[start], start, 0.0)
        for i in range(min(start, len(pts) - 2), len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            q = dx * dx + dy * dy
            u = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / q)) if q else 0.0
            qx, qy = a[0] + u * dx, a[1] + u * dy
            d = math.hypot(px - qx, py - qy)
            if d < best[0]:
                best = (d, (qx, qy), i, u)
        return best

    def walk(i, u, dist):
        a, b = pts[i], pts[i + 1]
        cx, cy = a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])
        while i < len(pts) - 1:
            nx, ny = pts[i + 1]
            L = math.hypot(nx - cx, ny - cy)
            if L >= dist and L > 1e-9:
                return (cx + dist * (nx - cx) / L, cy + dist * (ny - cy) / L)
            dist -= L
            cx, cy = nx, ny
            i += 1
        return pts[-1]

    def ang(err, speed, floor):
        nonlocal rot_dir
        if abs(err) < dead:
            rot_dir = 0.0
            return 0.0
        if rot_dir == 0.0 or err * rot_dir < 0.0:
            rot_dir = 1.0 if err > 0.0 else -1.0
        low = min(floor, speed)
        mag = speed if abs(err) >= align else max(low, speed * abs(err) / align)
        return rot_dir * mag

    while t < limit:
        if t + 1e-9 >= next_sensor:
            sx, sy, syaw = x, y, yaw
            next_sensor += 1.0 / sensor_hz
            d, cp, si, su = closest(sx, sy, idx)
            idx = max(idx, si)
            max_ct = max(max_ct, d)
            tgt = walk(si, su, lookahead)
            if math.hypot(tgt[0] - sx, tgt[1] - sy) > 2.0 * lookahead:
                tgt = cp
            desired = math.atan2(tgt[1] - sy, tgt[0] - sx)
            err = math.atan2(math.sin(desired - syaw), math.cos(desired - syaw))
            if abs(err) > align or state == 'ALIGN':
                if abs(err) <= aligned:
                    state = 'FOLLOW'
                    cmd = (v, 0.0)
                else:
                    state = 'ALIGN'
                    cmd = (0.0, ang(err, w_align, min_align))
            else:
                state = 'FOLLOW'
                cmd = (v, ang(err, w_fine, min_steer))
            sign = 1 if cmd[1] > 1e-9 else (-1 if cmd[1] < -1e-9 else 0)
            if last_sign and sign and sign != last_sign:
                reversals += 1
            if sign:
                last_sign = sign
        x += cmd[0] * math.cos(yaw) * dt
        y += cmd[0] * math.sin(yaw) * dt
        yaw += cmd[1] * gain * dt
        t += dt
        if math.hypot(x - pts[-1][0], y - pts[-1][1]) <= 0.12:
            return dict(ok=True, t=t, ct=max_ct, rev=reversals)
    return dict(ok=False, t=t, ct=max_ct, rev=reversals)


def final_yaw_sim(err0, slow_vel, slow_err, fast_vel, tol, pulse_off,
                  min_align=0.16, sensor_hz=3.5, dt=0.05, limit=30.0,
                  checker_tol=0.08, relaxed=0.20, timeout=8.0):
    """Endgame sim: does the robot stop within the checker's yaw tolerance?"""
    gain = 3.33
    yaw = 0.0
    target = err0
    seen = yaw
    t = 0.0
    next_sensor = 0.0
    tick = 0
    rot = 0.0
    pos_t = 0.0
    while t < limit:
        if t + 1e-9 >= next_sensor:
            seen = yaw
            next_sensor += 1.0 / sensor_hz
            err_c = math.atan2(math.sin(target - seen), math.cos(target - seen))
            if abs(err_c) <= (relaxed if (t - pos_t) >= timeout else checker_tol):
                return dict(ok=True, t=t, final=abs(err_c))
        err = math.atan2(math.sin(target - seen), math.cos(target - seen))
        cmd = 0.0
        if abs(err) > tol:
            if abs(err) <= slow_err:
                tick += 1
                if pulse_off and (tick % (1 + pulse_off)):
                    cmd = 0.0
                else:
                    if rot == 0.0 or err * rot < 0.0:
                        rot = 1.0 if err > 0 else -1.0
                    cmd = rot * max(min(slow_vel, min_align),
                                    slow_vel * abs(err) / 0.25) if abs(err) < 0.25 else rot * slow_vel
            else:
                tick = 0
                if rot == 0.0 or err * rot < 0.0:
                    rot = 1.0 if err > 0 else -1.0
                cmd = rot * fast_vel
        yaw += cmd * gain * dt
        t += dt
    err_c = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
    return dict(ok=False, t=t, final=abs(err_c))


def make_case(kind, res):
    size = round(6 / res)
    occ = np.zeros((size, size), np.int16)
    occ[:2, :] = occ[-2:, :] = occ[:, :2] = occ[:, -2:] = 100
    if kind == 'wall':
        x = round(3 / res)
        occ[round(1 / res):round(4.8 / res), x:x + 2] = 100
        return occ, (1., 3.), (5., 3.)
    if kind == 'slalom':
        occ[round(1 / res):round(4.2 / res), round(2 / res):round(2.15 / res)] = 100
        occ[round(1.8 / res):round(5 / res), round(4 / res):round(4.15 / res)] = 100
        return occ, (.7, 3.), (5.3, 3.)
    if kind == 'corridor':
        occ[:, :] = 100
        occ[round(2 / res):round(3 / res), round(.3 / res):round(5.7 / res)] = 0
        return occ, (.6, 2.5), (5.4, 2.5)
    occ[round(.5 / res):round(5.5 / res), round(2.8 / res):round(3 / res)] = 100
    occ[round(2.5 / res):round(3.5 / res), round(2.8 / res):round(3 / res)] = 0
    return occ, (1., 1.), (5., 5.)


fails = []
print('SYNTHETIC')
for res in (0.02, 0.03, 0.05, 0.10):
    for kind in ('wall', 'slalom', 'corridor', 'rooms'):
        occ, s, g = make_case(kind, res)
        edt = edt_for(occ, res)
        pl = plan(occ, edt, res, s, g)
        if not pl:
            fails.append(f'{kind}@{res}: no plan')
            continue
        sm = smooth(pl, edt, res, occ)
        det = (plan(occ, edt, res, s, g) == pl
               and smooth(pl, edt, res, occ) == sm)
        m = metrics(sm, occ, edt, 0.0, 0.0, res)
        sim = controller_sim(sm)
        print(f'  {kind}@{res:.2f} p={len(pl):3} s={len(sm):4} det={det} '
              f'clear={m["clear"]:.3f} gap={m["maxgap"]:.3f} '
              f'step={m["maxstep"]:5.1f}deg ctrl={sim["ok"]} '
              f'ct={sim["ct"]:.3f} rev={sim["rev"]:3}')
        if not det:
            fails.append(f'{kind}@{res}: nondeterministic')
        if m['bad']:
            fails.append(f'{kind}@{res}: {m["bad"]} collision samples')
        if m['clear'] + res * 1.5 < CLEAR:
            fails.append(f'{kind}@{res}: clearance {m["clear"]:.3f}')
        if m['maxgap'] > 0.08:
            fails.append(f'{kind}@{res}: gap {m["maxgap"]:.3f}')
        if not sim['ok']:
            fails.append(f'{kind}@{res}: controller did not reach goal')


def load_map(yp):
    d = yaml.safe_load(open(yp))
    res = float(d['resolution']); ox, oy = d['origin'][:2]
    pix = np.array(Image.open(Path(yp).parent / d['image']).convert('L'))
    occ = np.full(pix.shape, -1, np.int16); prob = (255 - pix) / 255.
    occ[prob >= float(d['occupied_thresh'])] = 100
    occ[prob <= float(d['free_thresh'])] = 0
    return occ, res, ox, oy


print('\nREAL MAPS')
for yp in sorted(MAPS.glob('*/map.yaml')):
    occ, res, ox, oy = load_map(yp)
    edt = edt_for(occ, res)
    lab, n = label((edt >= 0.20) & (occ == 0))
    ids, cnt = np.unique(lab[lab > 0], return_counts=True)
    ys, xs = np.where(lab == int(ids[np.argmax(cnt)]))
    rng = random.Random(23)
    ok = worst_step = 0
    minclear = 9.0
    maxgap = 0.0
    reach = 0
    ct = 0.0
    for _ in range(10):
        while True:
            ia, ib = rng.randrange(len(xs)), rng.randrange(len(xs))
            s = (ox + (xs[ia] + .5) * res, oy + (ys[ia] + .5) * res)
            g = (ox + (xs[ib] + .5) * res, oy + (ys[ib] + .5) * res)
            if math.dist(s, g) > 1.5:
                break
        pl = plan(occ, edt, res, s, g, ox, oy)
        if not pl:
            fails.append(f'{yp.parent.name}: no plan')
            continue
        sm = smooth(pl, edt, res, occ, ox, oy)
        m = metrics(sm, occ, edt, ox, oy, res)
        sim = controller_sim(sm)
        ok += 1
        reach += 1 if sim['ok'] else 0
        ct = max(ct, sim['ct'])
        worst_step = max(worst_step, m['maxstep'])
        minclear = min(minclear, m['clear'])
        maxgap = max(maxgap, m['maxgap'])
        if m['bad']:
            fails.append(f'{yp.parent.name}: {m["bad"]} collision samples')
        if not sim['ok']:
            fails.append(f'{yp.parent.name}: controller did not reach goal')
    print(f'  {yp.parent.name:10} res={res:.3f} plans={ok}/10 ctrl_reached={reach}/{ok} '
          f'minclear={minclear:.3f} maxgap={maxgap:.3f} '
          f'worst_step={worst_step:.1f}deg max_crosstrack={ct:.3f}')

print('\nFINAL YAW ENDGAME (pose rate 3.5 Hz as in nav.log)')
print(f'  {"start err":>9} | {"old law":>18} | {"new law":>18}')
for err0 in (0.20, 0.35, 0.60, 1.20, 2.74):
    old = final_yaw_sim(err0, slow_vel=0.30, slow_err=0.0, fast_vel=0.30,
                        tol=0.04, pulse_off=0, checker_tol=0.15, relaxed=0.15,
                        timeout=1e9)
    new = final_yaw_sim(err0, slow_vel=0.10, slow_err=0.35, fast_vel=0.30,
                        tol=0.05, pulse_off=1, checker_tol=0.08, relaxed=0.20,
                        timeout=8.0)
    print(f'  {err0:9.2f} | reached={str(old["ok"]):5} '
          f'err={math.degrees(old["final"]):5.1f}deg t={old["t"]:4.1f}s | '
          f'reached={str(new["ok"]):5} err={math.degrees(new["final"]):5.1f}deg '
          f't={new["t"]:4.1f}s')
    if not new['ok']:
        fails.append(f'final yaw {err0}: did not settle')

print(f'\nFAILURES: {len(fails)}')
for f in fails:
    print('  -', f)
