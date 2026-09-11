#!/usr/bin/env python3
"""Map project storage under ~/skill23_ws/studio_maps/<name>/."""

from __future__ import annotations

import json
import os
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore


STUDIO_MAPS = Path(os.path.expanduser('~/skill23_ws/studio_maps'))

# The bridge serves requests on threads: paint, zone save and deploy burn can
# all land at once. Every read-modify-write of map.pgm holds this.
_MAP_LOCK = threading.RLock()


def ensure_root() -> Path:
    STUDIO_MAPS.mkdir(parents=True, exist_ok=True)
    return STUDIO_MAPS


def project_dir(name: str) -> Path:
    safe = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name.strip())
    if not safe:
        raise ValueError('invalid map name')
    return ensure_root() / safe


def list_projects() -> List[Dict[str, Any]]:
    ensure_root()
    out = []
    for d in sorted(STUDIO_MAPS.iterdir()):
        if not d.is_dir():
            continue
        if d.name.startswith('_'):
            continue
        meta_path = d / 'meta.json'
        meta: Dict[str, Any] = {'name': d.name}
        if meta_path.is_file():
            try:
                meta.update(json.loads(meta_path.read_text()))
            except Exception:
                pass
        meta['path'] = str(d / 'map.yaml')
        meta['has_map'] = (d / 'map.yaml').is_file() and (d / 'map.pgm').is_file()
        out.append(meta)
    return out


def delete_project(name: str) -> None:
    """Remove a studio map project directory. Raises PermissionError with fix hint."""
    d = project_dir(name)
    if not d.is_dir():
        raise FileNotFoundError(f'project not found: {name}')
    try:
        shutil.rmtree(d)
    except PermissionError as exc:
        raise PermissionError(
            f'Cannot delete {d}: owned by another user (often root from sudo map save).\n'
            f'Fix once in a terminal:\n'
            f'  sudo chown -R $USER:$USER ~/skill23_ws/studio_maps\n'
            f'Then delete again, or:\n'
            f'  sudo rm -rf {d}'
        ) from exc


def write_meta(name: str, **kwargs: Any) -> Path:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    meta_path = d / 'meta.json'
    meta: Dict[str, Any] = {}
    if meta_path.is_file():
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            meta = {}
    meta.update(kwargs)
    meta['name'] = d.name
    meta['updated_at'] = time.time()
    meta_path.write_text(json.dumps(meta, indent=2))
    return meta_path


def map_prefix(name: str) -> str:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    return str(d / 'map')


def map_yaml_path(name: str) -> Path:
    return project_dir(name) / 'map.yaml'


def save_waypoints(name: str, waypoints: List[Dict[str, Any]]) -> Path:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    path = d / 'waypoints.json'
    path.write_text(json.dumps({'waypoints': waypoints}, indent=2))
    return path


def load_waypoints(name: str) -> List[Dict[str, Any]]:
    path = project_dir(name) / 'waypoints.json'
    if not path.is_file():
        return []
    data = json.loads(path.read_text())
    return list(data.get('waypoints') or [])


def save_sequences(name: str, sequences: Dict[str, Any]) -> Path:
    """Save named sequences (0-based WP indices) next to waypoints.json."""
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    path = d / 'sequences.json'
    clean: Dict[str, List[int]] = {}
    raw = sequences.get('sequences') if isinstance(sequences, dict) and 'sequences' in sequences else sequences
    if isinstance(raw, dict):
        for key, idxs in raw.items():
            if isinstance(idxs, list) and idxs:
                clean[str(key)] = [int(i) for i in idxs]
    path.write_text(json.dumps({'sequences': clean}, indent=2))
    return path


def load_sequences(name: str) -> Dict[str, List[int]]:
    path = project_dir(name) / 'sequences.json'
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text())
    except Exception:
        return {}
    raw = data.get('sequences') if isinstance(data, dict) else {}
    out: Dict[str, List[int]] = {}
    if isinstance(raw, dict):
        for key, idxs in raw.items():
            if isinstance(idxs, list) and idxs:
                out[str(key)] = [int(i) for i in idxs]
    return out


def commission_config_path(name: str) -> Path:
    return project_dir(name) / 'commission.json'


def load_commission_config(name: str) -> Dict[str, Any]:
    from skill23_studio.commission_mission import default_config, normalize_config

    path = commission_config_path(name)
    if not path.is_file():
        return default_config()
    try:
        data = json.loads(path.read_text())
    except Exception:
        return default_config()
    return normalize_config(data)


def save_commission_config(name: str, config: Dict[str, Any]) -> Path:
    from skill23_studio.commission_mission import normalize_config

    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    path = commission_config_path(name)
    path.write_text(json.dumps(normalize_config(config), indent=2))
    return path


def save_restrictions(name: str, rects: List[Dict[str, Any]]) -> Path:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    with _MAP_LOCK:
        # Snapshot the un-burned map *before* the zone list changes, so a legacy
        # in-place burn can still be undone from the zones it was made from.
        try:
            ensure_base_snapshot(name)
        except Exception:
            pass
        path = d / 'restrictions.json'
        path.write_text(json.dumps({'rectangles': rects}, indent=2))
    return path


def load_restrictions(name: str) -> List[Dict[str, Any]]:
    path = project_dir(name) / 'restrictions.json'
    if not path.is_file():
        return []
    data = json.loads(path.read_text())
    return list(data.get('rectangles') or [])


def _load_pgm(path: Path) -> Tuple[int, int, np.ndarray]:
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic not in (b'P5', b'P2'):
            raise ValueError(f'unsupported PGM: {magic!r}')
        line = f.readline()
        while line.startswith(b'#') or not line.strip():
            line = f.readline()
        w, h = [int(x) for x in line.split()]
        maxval = int(f.readline())
        if magic == b'P5':
            # frombuffer is read-only — copy so edits can assign
            raw = np.frombuffer(f.read(w * h), dtype=np.uint8).copy()
        else:
            raw = np.array([int(x) for x in f.read().split()], dtype=np.uint8)
        img = raw.reshape((h, w)).copy()
        if maxval != 255:
            img = (img.astype(np.float64) * (255.0 / maxval)).astype(np.uint8)
        return w, h, img


def _save_pgm(path: Path, img: np.ndarray) -> None:
    """Atomic write — map_server/world_model must never read a half file."""
    h, w = img.shape
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('wb') as f:
        f.write(b'P5\n')
        f.write(f'{w} {h}\n'.encode())
        f.write(b'255\n')
        f.write(img.astype(np.uint8).tobytes())
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _map_meta(name: str) -> Dict[str, Any]:
    """resolution + origin from map.yaml (yaml module optional)."""
    yaml_path = project_dir(name) / 'map.yaml'
    meta: Dict[str, Any] = {}
    if not yaml_path.is_file():
        return meta
    if yaml is not None:
        with yaml_path.open('r') as f:
            meta = yaml.safe_load(f) or {}
    else:
        for line in yaml_path.read_text().splitlines():
            if ':' in line and not line.strip().startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()
    return meta


def _res_origin(meta: Dict[str, Any]) -> Tuple[float, float, float]:
    res = float(meta.get('resolution', 0.05))
    origin = meta.get('origin', [0, 0, 0])
    if isinstance(origin, str):
        origin = [float(x) for x in origin.strip('[]').split(',')]
    return res, float(origin[0]), float(origin[1])


def _rect_mask(
    h: int,
    w: int,
    rects: List[Dict[str, Any]],
    res: float,
    ox: float,
    oy: float,
) -> np.ndarray:
    """Boolean PGM-row-order mask of all cells covered by the rectangles."""
    mask = np.zeros((h, w), dtype=bool)
    if not rects:
        return mask
    # Cell centres. PGM row0 = top = highest grid y.
    gx = np.arange(w, dtype=np.float64)
    gy = (h - 1) - np.arange(h, dtype=np.float64)
    wx = ox + (gx + 0.5) * res
    wy = oy + (gy + 0.5) * res
    for r in rects:
        cx = float(r.get('cx', 0.0))
        cy = float(r.get('cy', 0.0))
        rw = float(r.get('w', 0.0))
        rh = float(r.get('h', 0.0))
        if rw <= 0.0 or rh <= 0.0:
            continue
        yaw = float(r.get('yaw', 0.0))
        c, s = np.cos(yaw), np.sin(yaw)
        dx = (wx - cx)[None, :]
        dy = (wy - cy)[:, None]
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        mask |= (np.abs(lx) <= rw * 0.5) & (np.abs(ly) <= rh * 0.5)
    return mask


def _base_pgm_path(name: str) -> Path:
    return project_dir(name) / 'map_base.pgm'


def ensure_base_snapshot(name: str) -> Optional[Path]:
    """Keep map_base.pgm — the map *without* any restriction zones burned in.

    Zones are virtual walls: they must be removable. Burning used to edit
    map.pgm in place, so a deleted zone left its black cells behind forever
    (and the next burn painted the box back). Every burn now rebuilds map.pgm
    from this snapshot instead.

    For projects burned by the old code the snapshot does not exist yet, so it
    is created by clearing the cells of the zones saved at that moment — those
    are exactly the cells a burn could have blackened.
    """
    d = project_dir(name)
    pgm = d / 'map.pgm'
    base = _base_pgm_path(name)
    with _MAP_LOCK:
        if base.is_file():
            return base
        if not pgm.is_file():
            return None
        w, h, img = _load_pgm(pgm)
        rects = load_restrictions(name)
        if rects:
            res, ox, oy = _res_origin(_map_meta(name))
            # Only dark cells — a burn can only have blackened; unknown (grey)
            # cells inside a zone were never touched and must stay unknown.
            mask = _rect_mask(h, w, rects, res, ox, oy) & (img < 50)
            img[mask] = 254
        _save_pgm(base, img)
    return base


def invalidate_base_snapshot(name: str) -> None:
    """Drop the snapshot — call whenever map.pgm is replaced (save / import)."""
    try:
        _base_pgm_path(name).unlink()
    except FileNotFoundError:
        pass
    except Exception:
        pass


# Offline telemetry rebuilds this payload ~3x/s — cache it per file version.
_GRID_CACHE: Dict[str, Tuple[Tuple[Any, ...], Dict[str, Any]]] = {}


def apply_occupancy_edits(
    name: str,
    ops: List[Dict[str, Any]],
) -> None:
    """Apply wall paint/erase ops on map.pgm.

    Each op: {action: 'wall'|'erase', cells: [[gx,gy], ...]}
    PGM: 0=occupied (black), 254=free (white) in ROS convention after flip —
    we match world_model: dark = occupied.

    Ops are also applied to map_base.pgm so hand edits survive a re-burn.
    """
    d = project_dir(name)
    pgm = d / 'map.pgm'
    if not pgm.is_file():
        raise FileNotFoundError('map.pgm missing')
    with _MAP_LOCK:
        base = ensure_base_snapshot(name)
        targets = [pgm] + ([base] if base and base.is_file() else [])
        for target in targets:
            w, h, img = _load_pgm(target)
            for op in ops:
                action = str(op.get('action', ''))
                if action not in ('wall', 'erase'):
                    continue
                val = 0 if action == 'wall' else 254
                for cell in op.get('cells') or []:
                    gx, gy = int(cell[0]), int(cell[1])
                    # OccupancyGrid row0 = bottom; PGM row0 = top → flip Y
                    row = h - 1 - gy
                    col = gx
                    if not (0 <= col < w and 0 <= row < h):
                        continue
                    img[row, col] = val
            try:
                _save_pgm(target, img)
            except PermissionError as exc:
                raise PermissionError(
                    f'Cannot write {target} (likely root-owned). Run:\n'
                    f'  sudo chown -R $USER:$USER ~/skill23_ws/studio_maps'
                ) from exc
        # Drop UI grid cache so the next telemetry frame reloads map.pgm.
        _GRID_CACHE.pop(str(d), None)


def burn_restrictions_into_map(name: str) -> None:
    """Rebuild map.pgm as base map + current restriction zones.

    Idempotent: burning twice changes nothing, and removing a zone restores
    the cells it had blackened.
    """
    d = project_dir(name)
    yaml_path = d / 'map.yaml'
    pgm = d / 'map.pgm'
    if not yaml_path.is_file() or not pgm.is_file():
        return
    with _MAP_LOCK:
        base = ensure_base_snapshot(name)
        src = base if base and base.is_file() else pgm
        rects = load_restrictions(name)
        w, h, img = _load_pgm(src)
        if rects:
            res, ox, oy = _res_origin(_map_meta(name))
            img[_rect_mask(h, w, rects, res, ox, oy)] = 0
        try:
            _save_pgm(pgm, img)
        except PermissionError as exc:
            raise PermissionError(
                f'Cannot write {pgm} (likely root-owned). Run:\n'
                f'  sudo chown -R $USER:$USER ~/skill23_ws/studio_maps'
            ) from exc
        _GRID_CACHE.pop(str(d), None)


# load_map_grid uses _GRID_CACHE (defined above apply_occupancy_edits).


def load_map_grid(name: str) -> Optional[Dict[str, Any]]:
    """Load map.pgm + map.yaml as an occupancy-like payload for the UI."""
    d = project_dir(name)
    yaml_path = d / 'map.yaml'
    pgm = d / 'map.pgm'
    if not yaml_path.is_file() or not pgm.is_file():
        return None
    try:
        st_p = pgm.stat()
        st_y = yaml_path.stat()
        key = (st_p.st_mtime_ns, st_p.st_size, st_y.st_mtime_ns, st_y.st_size)
    except OSError:
        key = None
    cached = _GRID_CACHE.get(str(d))
    if key is not None and cached is not None and cached[0] == key:
        return cached[1]

    res, ox, oy = _res_origin(_map_meta(name))
    w, h, img = _load_pgm(pgm)
    # OccupancyGrid row0 = bottom, PGM row0 = top → flip.
    flipped = img[::-1, :]
    # PGM trinary: 0=occupied, 254=free, 205=unknown — do not treat 205 as free.
    grid = np.full(flipped.shape, -1, dtype=np.int16)
    grid[flipped <= 10] = 100
    grid[flipped >= 250] = 0
    payload = {
        'width': w,
        'height': h,
        'resolution': res,
        'origin': {'x': ox, 'y': oy},
        'data': grid.reshape(-1).tolist(),
    }
    if key is not None:
        _GRID_CACHE[str(d)] = (key, payload)
    return payload


def slam_params_path(name: str) -> Path:
    return project_dir(name) / 'slam_params.yaml'


def _default_slam_params_src() -> Path:
    candidates = [
        Path(os.path.expanduser('~/skill23_ws/src/studica_ws/src/studica_control/config/slam_params.yaml')),
        Path(__file__).resolve().parents[3] / 'studica_ws' / 'src' / 'studica_control' / 'config' / 'slam_params.yaml',
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('studica_control'))
        candidates.insert(0, share / 'config' / 'slam_params.yaml')
    except Exception:
        pass
    for p in candidates:
        if p.is_file():
            return p
    raise FileNotFoundError('studica_control slam_params.yaml not found')


def write_slam_params(
    name: str,
    *,
    resolution: float = 0.03,
    max_laser_range: float = 10.0,
    min_laser_range: float = 0.05,
) -> Path:
    """Write per-project slam_toolbox YAML with map grid overrides."""
    resolution = float(resolution)
    max_laser_range = float(max_laser_range)
    min_laser_range = float(min_laser_range)
    if not (0.01 <= resolution <= 0.2):
        raise ValueError('resolution must be between 0.01 and 0.2 m')
    if not (1.0 <= max_laser_range <= 30.0):
        raise ValueError('max_laser_range must be between 1 and 30 m')
    if not (0.0 <= min_laser_range < max_laser_range):
        raise ValueError('invalid min_laser_range')

    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    src = _default_slam_params_src()
    text = src.read_text()

    if yaml is not None:
        data = yaml.safe_load(text) or {}
        params = data.setdefault('slam_toolbox', {}).setdefault('ros__parameters', {})
        params['resolution'] = resolution
        params['max_laser_range'] = max_laser_range
        params['min_laser_range'] = min_laser_range
        out = d / 'slam_params.yaml'
        with out.open('w') as f:
            yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    else:
        # Fallback: line replace
        lines = []
        for line in text.splitlines():
            stripped = line.lstrip()
            indent = line[: len(line) - len(stripped)]
            if stripped.startswith('resolution:'):
                lines.append(f'{indent}resolution: {resolution}')
            elif stripped.startswith('max_laser_range:'):
                lines.append(f'{indent}max_laser_range: {max_laser_range}')
            elif stripped.startswith('min_laser_range:'):
                lines.append(f'{indent}min_laser_range: {min_laser_range}')
            else:
                lines.append(line)
        out = d / 'slam_params.yaml'
        out.write_text('\n'.join(lines) + '\n')

    write_meta(
        name,
        resolution=resolution,
        max_laser_range=max_laser_range,
        min_laser_range=min_laser_range,
    )
    return out


def import_map_files(
    name: str,
    yaml_src: str,
    pgm_src: Optional[str] = None,
    robot: str = 'grip',
) -> Path:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    src_y = Path(os.path.expanduser(yaml_src))
    if not src_y.is_file():
        raise FileNotFoundError(str(src_y))
    shutil.copy2(src_y, d / 'map.yaml')
    if pgm_src:
        src_p = Path(os.path.expanduser(pgm_src))
    else:
        # resolve image: from yaml
        img_name = 'map.pgm'
        for line in src_y.read_text().splitlines():
            if line.strip().startswith('image:'):
                img_name = line.split(':', 1)[1].strip().strip('"\'')
                break
        src_p = src_y.parent / img_name
    if not src_p.is_file():
        raise FileNotFoundError(f'PGM not found: {src_p}')
    shutil.copy2(src_p, d / 'map.pgm')
    # rewrite yaml image: to local basename
    text = (d / 'map.yaml').read_text()
    lines = []
    for line in text.splitlines():
        if line.strip().startswith('image:'):
            lines.append('image: map.pgm')
        else:
            lines.append(line)
    (d / 'map.yaml').write_text('\n'.join(lines) + '\n')
    # New pixels — any zone snapshot from a previous map is meaningless now.
    invalidate_base_snapshot(name)
    # Stale restriction boxes from a previous project would burn/erase the
    # newly imported walls on the first zone sync — clear them on import.
    rest = d / 'restrictions.json'
    try:
        rest.write_text('[]\n')
    except OSError:
        pass
    write_meta(name, robot=robot, imported_from=str(src_y), created_at=time.time())
    return d / 'map.yaml'


def gc_walls_path(name: str) -> Path:
    return project_dir(name) / 'gc_walls.json'


def load_gc_walls(name: str) -> List[List[int]]:
    """Return [[gx, gy], ...] wall cells for GC planner overlay (not map.pgm)."""
    path = gc_walls_path(name)
    if not path.is_file():
        return []
    try:
        raw = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return []
    cells: List[List[int]] = []
    if isinstance(raw, dict):
        raw = raw.get('cells') or []
    if not isinstance(raw, list):
        return []
    for cell in raw:
        try:
            cells.append([int(cell[0]), int(cell[1])])
        except (TypeError, ValueError, IndexError):
            continue
    return cells


def save_gc_walls(name: str, cells: List[List[int]]) -> Path:
    d = project_dir(name)
    d.mkdir(parents=True, exist_ok=True)
    path = gc_walls_path(name)
    uniq = sorted({(int(c[0]), int(c[1])) for c in cells if len(c) >= 2})
    path.write_text(json.dumps({
        'cells': [[gx, gy] for gx, gy in uniq],
        'updated_at': time.time(),
    }, indent=2) + '\n')
    return path
