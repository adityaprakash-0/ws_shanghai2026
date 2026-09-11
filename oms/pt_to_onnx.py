from ultralytics import YOLO

PT_PATH = "/home/sr06/oms/best.pt"

model = YOLO(PT_PATH)

# Use the image size the model was actually trained at, falling back to 640
train_args = getattr(model, "ckpt", {}).get("train_args", {}) or {}
imgsz = train_args.get("imgsz", 640)
print(f"Exporting at imgsz={imgsz}")

onnx_path = model.export(
    format="onnx",
    opset=13,
    imgsz=imgsz,
    simplify=True,
    dynamic=False,   # set True if your runtime needs variable batch/input size
)

print(f"Saved: {onnx_path}")