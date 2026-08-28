"""YOLOv8n NCNN detector — loaded only when Detect+Align starts.

Idle callers never construct this class. Inference uses Tencent ncnn
(no PyTorch / Ultralytics at runtime).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import yaml


@dataclass
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    conf: float
    class_id: int
    name: str = ''

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def area(self) -> float:
        return max(0.0, self.x2 - self.x1) * max(0.0, self.y2 - self.y1)


def _nms(dets: List[Detection], iou_thr: float) -> List[Detection]:
    if not dets:
        return []
    order = sorted(dets, key=lambda d: d.conf, reverse=True)
    keep: List[Detection] = []
    while order:
        best = order.pop(0)
        keep.append(best)
        remain: List[Detection] = []
        for det in order:
            ix1 = max(best.x1, det.x1)
            iy1 = max(best.y1, det.y1)
            ix2 = min(best.x2, det.x2)
            iy2 = min(best.y2, det.y2)
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            union = best.area + det.area - inter
            if union <= 1e-9 or (inter / union) <= iou_thr:
                remain.append(det)
        order = remain
    return keep


def _letterbox(rgb: np.ndarray, size: int, pad: int = 114) -> Tuple[np.ndarray, float, int, int]:
    import cv2
    h, w = rgb.shape[:2]
    scale = min(size / float(h), size / float(w))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    resized = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), pad, dtype=np.uint8)
    dx = (size - nw) // 2
    dy = (size - nh) // 2
    canvas[dy:dy + nh, dx:dx + nw] = resized
    return canvas, scale, dx, dy


class YoloV8Ncnn:
    """Ultralytics-exported YOLOv8n (in0 / out0, 84 x N)."""

    def __init__(
            self, model_dir: str, *,
            input_size: int = 320,
            conf_threshold: float = 0.35,
            nms_iou: float = 0.45,
            num_threads: int = 2) -> None:
        self.model_dir = os.path.abspath(model_dir)
        self.input_size = int(input_size)
        self.conf_threshold = float(conf_threshold)
        self.nms_iou = float(nms_iou)
        self.num_threads = max(1, int(num_threads))
        self.names: dict = {}
        self._net = None
        self._load()

    def _load(self) -> None:
        import ncnn
        param = os.path.join(self.model_dir, 'model.ncnn.param')
        binary = os.path.join(self.model_dir, 'model.ncnn.bin')
        meta = os.path.join(self.model_dir, 'metadata.yaml')
        if not os.path.isfile(param) or not os.path.isfile(binary):
            raise FileNotFoundError(
                f'NCNN YOLOv8n model missing in {self.model_dir} '
                '(need model.ncnn.param + model.ncnn.bin)')
        if os.path.isfile(meta):
            with open(meta, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            raw = data.get('names') or {}
            if isinstance(raw, dict):
                self.names = {int(k): str(v) for k, v in raw.items()}
            elif isinstance(raw, list):
                self.names = {i: str(n) for i, n in enumerate(raw)}
            imgsz = data.get('imgsz')
            if isinstance(imgsz, (list, tuple)) and imgsz:
                self.input_size = int(imgsz[0])
        net = ncnn.Net()
        net.opt.use_vulkan_compute = False
        net.opt.num_threads = self.num_threads
        try:
            net.opt.use_packing_layout = True
        except Exception:
            pass
        if net.load_param(param) != 0:
            raise RuntimeError(f'ncnn load_param failed: {param}')
        if net.load_model(binary) != 0:
            raise RuntimeError(f'ncnn load_model failed: {binary}')
        self._net = net

    def close(self) -> None:
        self._net = None

    def name_of(self, class_id: int) -> str:
        return str(self.names.get(int(class_id), str(class_id)))

    def class_id_for_name(self, name: str) -> Optional[int]:
        want = str(name or '').strip().lower()
        if not want:
            return None
        for cid, label in self.names.items():
            if str(label).strip().lower() == want:
                return int(cid)
        return None

    def detect(self, rgb: np.ndarray) -> List[Detection]:
        import ncnn
        if rgb is None or rgb.size == 0 or self._net is None:
            return []
        if rgb.ndim != 3 or rgb.shape[2] < 3:
            return []
        img = np.ascontiguousarray(rgb[:, :, :3], dtype=np.uint8)
        h, w = img.shape[:2]
        blob, scale, dx, dy = _letterbox(img, self.input_size)
        mat_in = ncnn.Mat.from_pixels(
            blob, ncnn.Mat.PixelType.PIXEL_RGB,
            self.input_size, self.input_size)
        mat_in.substract_mean_normalize([0.0, 0.0, 0.0], [1 / 255.0, 1 / 255.0, 1 / 255.0])
        ex = self._net.create_extractor()
        ex.input('in0', mat_in)
        _, out0 = ex.extract('out0')
        arr = np.array(out0)
        if arr.ndim == 3:
            arr = arr[0]
        if arr.ndim != 2:
            return []
        # Ultralytics: (84, N) = xywh + 80 class scores, letterbox pixels.
        if arr.shape[0] < arr.shape[1]:
            feat, num = arr.shape
        else:
            arr = arr.T
            feat, num = arr.shape
        if feat < 5 or num <= 0:
            return []
        xywh = arr[:4]
        scores = arr[4:]
        cls_id = np.argmax(scores, axis=0)
        cls_sc = scores[cls_id, np.arange(num)]
        dets: List[Detection] = []
        for i in range(num):
            conf = float(cls_sc[i])
            if conf < self.conf_threshold:
                continue
            cx = (float(xywh[0, i]) - dx) / scale
            cy = (float(xywh[1, i]) - dy) / scale
            bw = float(xywh[2, i]) / scale
            bh = float(xywh[3, i]) / scale
            x1 = max(0.0, min(float(w), cx - bw * 0.5))
            y1 = max(0.0, min(float(h), cy - bh * 0.5))
            x2 = max(0.0, min(float(w), cx + bw * 0.5))
            y2 = max(0.0, min(float(h), cy + bh * 0.5))
            if x2 - x1 < 2.0 or y2 - y1 < 2.0:
                continue
            cid = int(cls_id[i])
            dets.append(Detection(
                x1, y1, x2, y2, conf, cid, self.name_of(cid)))
        return _nms(dets, self.nms_iou)

    def pick(
            self, dets: Sequence[Detection], *,
            class_id: int = -1,
            class_name: str = '',
            min_area: float = 0.0) -> Optional[Detection]:
        want_id = int(class_id)
        if want_id < 0 and class_name:
            mapped = self.class_id_for_name(class_name)
            if mapped is not None:
                want_id = mapped
        best: Optional[Detection] = None
        for det in dets:
            if det.area < min_area:
                continue
            if want_id >= 0 and int(det.class_id) != want_id:
                continue
            if best is None or det.conf > best.conf:
                best = det
        return best
