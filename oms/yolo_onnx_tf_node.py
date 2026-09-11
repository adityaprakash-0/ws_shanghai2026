#!/usr/bin/env python3
"""
ROS 2 node: YOLO ONNX detection on /camera/color/image_raw + a TF frame at the
3D centre of every detected object.

Model (from best.onnx metadata):
    input  : images  [1, 3, 416, 416]  float32, RGB, 0-1, NCHW
    output : output0 [1, 8, 3549]      -> 4 box (cx,cy,w,h) + 4 class scores
    classes: 0 background, 1 Cylinder, 2 Rectangle, 3 Semisphere
    NMS    : not in the graph, done here.

A bounding box alone only gives a ray, not a position, so the 3D centre comes
from the aligned depth image + camera_info intrinsics. If you have no depth
sensor, set use_depth:=false and the node places each object on a fixed plane
at assumed_depth metres along its ray (direction correct, range guessed).

Run (RealSense defaults):
    python3 yolo_onnx_tf_node.py --ros-args \
        -p model_path:=/abs/path/best.onnx \
        -p image_topic:=/camera/color/image_raw \
        -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
        -p camera_info_topic:=/camera/color/camera_info
"""

import cv2
import message_filters
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

CLASS_NAMES = ["background", "Cylinder", "Rectangle", "Semisphere"]


def letterbox(img, new_shape=(416, 416), color=(114, 114, 114)):
    """Resize keeping aspect ratio, pad to new_shape. Returns img, ratio, (dw, dh)."""
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    dh, dw = (new_shape[0] - nh) / 2, (new_shape[1] - nw) / 2
    if (h, w) != (nh, nw):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                             cv2.BORDER_CONSTANT, value=color)
    return img, r, (left, top)


def sample_depth(depth, u, v, patch=5):
    """Median of the valid depth pixels in a patch around (u, v). Returns metres or None.

    Object centres often land on a hole (specular, thin, or edge pixels read 0),
    so a single pixel lookup is unreliable; the median of the neighbourhood also
    rejects background pixels bleeding in at the box centre.
    """
    h, w = depth.shape[:2]
    k = patch // 2
    y0, y1 = max(0, v - k), min(h, v + k + 1)
    x0, x1 = max(0, u - k), min(w, u + k + 1)
    win = depth[y0:y1, x0:x1].astype(np.float32)
    if win.size == 0:
        return None

    if depth.dtype == np.uint16:      # 16UC1, millimetres (RealSense default)
        win = win / 1000.0
    # 32FC1 is already metres

    valid = win[np.isfinite(win) & (win > 0.05) & (win < 20.0)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def sanitize(name):
    return "".join(c if c.isalnum() or c in "_-" else "_" for c in name)


class YoloTfNode(Node):
    def __init__(self):
        super().__init__("yolo_onnx_tf_node")

        self.declare_parameter("model_path", "best.onnx")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("skip_background_class", True)
        self.declare_parameter("use_gpu", False)
        self.declare_parameter("publish_annotated", True)
        # TF options
        self.declare_parameter("use_depth", True)
        self.declare_parameter("assumed_depth", 0.5)        # metres, only if use_depth=false
        self.declare_parameter("depth_patch", 5)            # px window for the median
        self.declare_parameter("frame_prefix", "obj_")
        self.declare_parameter("parent_frame", "")          # "" = use image header.frame_id
        self.declare_parameter("sync_slop", 0.05)           # s, colour/depth pairing

        p = self.get_parameter
        model_path = p("model_path").value
        self.conf_th = float(p("conf_threshold").value)
        self.iou_th = float(p("iou_threshold").value)
        self.skip_bg = bool(p("skip_background_class").value)
        self.publish_annotated = bool(p("publish_annotated").value)
        self.use_depth = bool(p("use_depth").value)
        self.assumed_depth = float(p("assumed_depth").value)
        self.depth_patch = int(p("depth_patch").value)
        self.frame_prefix = p("frame_prefix").value
        self.parent_frame_override = p("parent_frame").value

        providers = ["CPUExecutionProvider"]
        if p("use_gpu").value:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        so = ort.SessionOptions()
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.session = ort.InferenceSession(model_path, so, providers=providers)
        self.input_name = self.session.get_inputs()[0].name
        ishape = self.session.get_inputs()[0].shape
        self.in_h = ishape[2] if isinstance(ishape[2], int) else 416
        self.in_w = ishape[3] if isinstance(ishape[3], int) else 416

        self.names = CLASS_NAMES
        meta = self.session.get_modelmeta().custom_metadata_map
        if "names" in meta:
            try:
                d = eval(meta["names"], {"__builtins__": {}})
                self.names = [d[i] for i in sorted(d)]
            except Exception:
                pass

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.K = None                  # (fx, fy, cx, cy)
        self.busy = False
        self._warned_no_depth = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        # Intrinsics are required to turn pixels into metres.
        self.create_subscription(CameraInfo, p("camera_info_topic").value,
                                 self.on_info, 10)

        if self.use_depth:
            sub_c = message_filters.Subscriber(self, Image, p("image_topic").value, qos_profile=qos)
            sub_d = message_filters.Subscriber(self, Image, p("depth_topic").value, qos_profile=qos)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [sub_c, sub_d], queue_size=5, slop=float(p("sync_slop").value))
            self.sync.registerCallback(self.on_pair)
        else:
            self.create_subscription(Image, p("image_topic").value,
                                     lambda m: self.on_pair(m, None), qos)

        self.pub_img = self.create_publisher(Image, "~/annotated", 1)
        self.pub_det = self.create_publisher(String, "~/detections", 10)

        self.get_logger().info(
            f"model={model_path} in={self.in_w}x{self.in_h} "
            f"providers={self.session.get_providers()} classes={self.names} "
            f"use_depth={self.use_depth}"
        )

    # ---------------- camera intrinsics ----------------
    def on_info(self, msg: CameraInfo):
        if self.K is None:
            k = msg.k
            self.K = (k[0], k[4], k[2], k[5])
            self.get_logger().info(
                f"intrinsics fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.2f} cy={k[5]:.2f} "
                f"({msg.width}x{msg.height}, frame={msg.header.frame_id})")

    def deproject(self, u, v, z):
        """Pixel + depth -> XYZ in the camera optical frame (x right, y down, z forward)."""
        fx, fy, cx, cy = self.K
        return ((u - cx) * z / fx, (v - cy) * z / fy, z)

    # ---------------- inference ----------------
    def preprocess(self, bgr):
        img, r, (dw, dh) = letterbox(bgr, (self.in_h, self.in_w))
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        blob = (rgb.astype(np.float32) / 255.0).transpose(2, 0, 1)[None]
        return np.ascontiguousarray(blob), r, dw, dh

    def postprocess(self, out, r, dw, dh, shape):
        pred = np.squeeze(out, 0).T
        boxes_cxcywh, scores_all = pred[:, :4], pred[:, 4:]

        if self.skip_bg and self.names and self.names[0].lower() == "background":
            scores_all = scores_all.copy()
            scores_all[:, 0] = 0.0

        cls_ids = np.argmax(scores_all, axis=1)
        scores = scores_all[np.arange(len(cls_ids)), cls_ids]
        keep = scores > self.conf_th
        if not np.any(keep):
            return []
        boxes_cxcywh, scores, cls_ids = boxes_cxcywh[keep], scores[keep], cls_ids[keep]

        cx, cy, w, h = boxes_cxcywh.T
        x1 = ((cx - w / 2 - dw) / r).clip(0, shape[1] - 1)
        x2 = ((cx + w / 2 - dw) / r).clip(0, shape[1] - 1)
        y1 = ((cy - h / 2 - dh) / r).clip(0, shape[0] - 1)
        y2 = ((cy + h / 2 - dh) / r).clip(0, shape[0] - 1)

        rects = np.stack([x1, y1, x2 - x1, y2 - y1], axis=1)
        idxs = cv2.dnn.NMSBoxes(rects.tolist(), scores.tolist(), self.conf_th, self.iou_th)
        if len(idxs) == 0:
            return []
        return [(float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i]),
                 float(scores[i]), int(cls_ids[i])) for i in np.array(idxs).flatten()]

    # ---------------- TF ----------------
    def publish_tfs(self, dets, depth, header):
        if self.K is None:
            self.get_logger().warn("no camera_info yet, cannot publish TF", once=True)
            return []

        parent = self.parent_frame_override or header.frame_id
        if not parent:
            self.get_logger().warn("image header.frame_id is empty; set parent_frame", once=True)
            return []

        transforms, labelled, per_class = [], [], {}
        for x1, y1, x2, y2, score, cid in dets:
            u = int(round((x1 + x2) / 2))
            v = int(round((y1 + y2) / 2))

            if depth is not None:
                z = sample_depth(depth, u, v, self.depth_patch)
                if z is None:
                    # No valid depth at the centre: skip rather than publish a
                    # frame at a made-up range.
                    self.get_logger().debug(f"no valid depth at ({u},{v}), skipping TF")
                    labelled.append((x1, y1, x2, y2, score, cid, None))
                    continue
            else:
                z = self.assumed_depth
                if not self._warned_no_depth:
                    self.get_logger().warn(
                        f"use_depth=false: TF range is the assumed {z} m, direction only")
                    self._warned_no_depth = True

            X, Y, Z = self.deproject(u, v, z)

            name = self.names[cid] if cid < len(self.names) else str(cid)
            idx = per_class.get(name, 0)
            per_class[name] = idx + 1
            child = f"{self.frame_prefix}{sanitize(name)}_{idx}"

            t = TransformStamped()
            t.header.stamp = header.stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = X
            t.transform.translation.y = Y
            t.transform.translation.z = Z
            t.transform.rotation.w = 1.0   # position only; box gives no orientation
            transforms.append(t)
            labelled.append((x1, y1, x2, y2, score, cid, (X, Y, Z)))

        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
        return labelled

    # ---------------- drawing ----------------
    def draw(self, frame, labelled):
        for x1, y1, x2, y2, score, cid, xyz in labelled:
            name = self.names[cid] if cid < len(self.names) else str(cid)
            color = ((37 * (cid + 1)) % 255, (17 * (cid + 3)) % 255, (97 * (cid + 5)) % 255)
            p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
            cv2.rectangle(frame, p1, p2, color, 2)
            cu, cv_ = int((x1 + x2) / 2), int((y1 + y2) / 2)
            cv2.drawMarker(frame, (cu, cv_), color, cv2.MARKER_CROSS, 14, 2)

            label = f"{name} {score:.2f}"
            if xyz:
                label += f" [{xyz[0]:+.2f},{xyz[1]:+.2f},{xyz[2]:.2f}]m"
            else:
                label += " no-depth"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (p1[0], p1[1] - th - 4), (p1[0] + tw, p1[1]), color, -1)
            cv2.putText(frame, label, (p1[0], p1[1] - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        return frame

    # ---------------- callback ----------------
    def on_pair(self, color_msg: Image, depth_msg):
        if self.busy:
            return
        self.busy = True
        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = None
            if depth_msg is not None:
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
                if depth.shape[:2] != frame.shape[:2]:
                    self.get_logger().warn(
                        f"depth {depth.shape[:2]} != colour {frame.shape[:2]}: "
                        "subscribe to the ALIGNED depth topic", once=True)
                    depth = None

            blob, r, dw, dh = self.preprocess(frame)
            out = self.session.run(None, {self.input_name: blob})[0]
            dets = self.postprocess(out, r, dw, dh, frame.shape)
            labelled = self.publish_tfs(dets, depth, color_msg.header)

            if labelled:
                txt = "; ".join(
                    f"{self.names[c] if c < len(self.names) else c}:{s:.2f}"
                    + (f"@({xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:.3f})m" if xyz else "@no-depth")
                    for _, _, _, _, s, c, xyz in labelled)
                self.get_logger().info(txt)
                m = String(); m.data = txt
                self.pub_det.publish(m)

            if self.publish_annotated:
                out_msg = self.bridge.cv2_to_imgmsg(self.draw(frame, labelled), encoding="bgr8")
                out_msg.header = color_msg.header
                self.pub_img.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"inference failed: {e}")
        finally:
            self.busy = False


def main():
    rclpy.init()
    node = YoloTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
