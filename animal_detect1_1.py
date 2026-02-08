import onnxruntime as ort
import numpy as np
import cv2
import time

# 模型路径（替换为你的实际路径）
MODEL_PATH = 'animal_detect.onnx'
INPUT_SIZE = 960  # 你训练和导出时设置的输入尺寸（如 640 或 960）

# 类别标签（与你训练时的 data.yaml 保持一致）
class_names = ['elephant', 'monkey', 'peacock', 'tiger', 'wolf']  # 替换为你的类别

# 创建 ONNX 推理会话
session = ort.InferenceSession(MODEL_PATH, providers=['CPUExecutionProvider'])

input_name = session.get_inputs()[0].name
output_name = session.get_outputs()[0].name

# 打开摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("failed to open camera")
    exit()

def preprocess(img, size):
    img = cv2.resize(img, (size, size))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC → CHW
    img = np.expand_dims(img, axis=0)  # 增加 batch 维度
    return img

def postprocess(prediction, img_shape, conf_thres=0.4, iou_thres=0.45):
    boxes = []
    img_h, img_w = img_shape
    for det in prediction:
        x1, y1, x2, y2, conf, cls = det[:6]
        if conf < conf_thres:
            continue
        x1 = int(x1 / INPUT_SIZE * img_w)
        x2 = int(x2 / INPUT_SIZE * img_w)
        y1 = int(y1 / INPUT_SIZE * img_h)
        y2 = int(y2 / INPUT_SIZE * img_h)
        boxes.append((class_names[int(cls)], conf, (x1, y1, x2, y2)))
    return boxes

print(" detection start ,press 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("failed to read frame from camera")
        break

    img_input = preprocess(frame, INPUT_SIZE)

    # 推理
    pred = session.run([output_name], {input_name: img_input})[0]

    # YOLOv5 ONNX 输出 shape: (1, num_detections, 6)
    pred = np.squeeze(pred, axis=0)

    detections = postprocess(pred, frame.shape[:2])

    for cls_name, conf, (x1, y1, x2, y2) in detections:
        print(f"find {cls_name}，position: ({x1},{y1}) - ({x2},{y2})，confidence: {conf:.2f}")
        # 可视化
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'{cls_name} {conf:.2f}', (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    cv2.imshow('YOLOv5 ONNX Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
