import cv2
import torch
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


# 加载模型（默认使用 yolov5s）
model = torch.hub.load('yolov5', 'custom', path='animal_detector.pt', source='local')
model.eval()

# 打开默认摄像头
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

print("开始识别，按 'q' 键退出...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法读取摄像头内容")
        break
        
    ## 创建副本用于颜色处理
    #frame = frame.astype(np.float32) 
    ## 增强绿色和红色通道，减少蓝色通道
    #frame[:, :, 0] *= 0.92   # B 通道（减少）
    #frame[:, :, 1] *= 1.1   # G 通道（增强）
    #frame[:, :, 2] *= 1.05  # R 通道（适当增强）    
    ## 再整体变暗
    #frame *= 0.8
    ## 限制值在 0~255 并转回 uint8
    #frame = np.clip(frame, 0, 255).astype(np.uint8)
    #
    ## alpha 是对比度系数，>1 表示增强对比度，beta 是亮度偏移（可选）
    #alpha = 1.7  # 对比度增强（1.0 = 原图）
    #beta = -60     # 亮度偏移
    #frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
    
    # 推理
    results = model(frame)

    # 获取预测结果（Pandas 格式，方便操作）
    detections = results.pandas().xyxy[0]

    # 打印每个识别结果的名称和位置信息
    for index, row in detections.iterrows():
        name = row['name']
        confidence = row['confidence']
        x1, y1, x2, y2 = map(int, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
        
        if(abs((x1-x2)*(y1 - y2)) > 6000):
            name = 'elephant'
        
        # 定义橙色范围
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([35, 255, 255])
        roi = tmpframe[y1:y2, x1:x2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        orange_ratio = np.sum(mask > 0) / (roi.shape[0] * roi.shape[1])
        if(orange_ratio > 0.01):
            name = 'tiger'
        
        if((x1+x2)/2 > 200 and (x1+x2)/2 < 400 and (y1+y2)/2 > 150 and (y1+y2)/2 < 300):
        
            print(f"检测到: {name}, 位置: ({x1}, {y1}), ({x2}, {y2}), 置信度: {confidence:.2f}")

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{name} {confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        

    # 显示图像
    cv2.imshow("YOLOv5 Real-Time Detection", frame)

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
