NCNN YOLOv8n (imgsz=320) used by studio_detect_align.

Files:
  model.ncnn.param
  model.ncnn.bin
  metadata.yaml

Re-export:
  python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='ncnn', imgsz=320)"
  then copy yolov8n_ncnn_model/{model.ncnn.param,model.ncnn.bin,metadata.yaml} here.
