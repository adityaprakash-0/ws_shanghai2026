import numpy as np
import onnx2tf
import onnx2tf.onnx2tf as _o2t

# onnx2tf downloads a 20x128x128x3 sample-image array for its accuracy check.
# That download is failing on this machine, so supply the array locally instead.
def _fake_test_image_data():
    rng = np.random.default_rng(0)
    return rng.random((20, 128, 128, 3), dtype=np.float32)

_o2t.download_test_image_data = _fake_test_image_data

onnx2tf.convert(
    input_onnx_file_path="/home/sr06/oms/best.onnx",
    output_folder_path="/home/sr06/oms/tflite_out",
    copy_onnx_input_output_names_to_tflite=True,
    non_verbose=False,
)

print("Done -> /home/sr06/oms/tflite_out")