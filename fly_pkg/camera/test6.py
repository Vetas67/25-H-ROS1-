import cv2
import torch
import numpy as np
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


REQUEST_CONFIDENCE = 0.4

# 加载模型（默认使用 yolov5s）
model = torch.hub.load('yolov5', 'custom', path='animal_detector.pt', source='local')
model.eval()

# 打开默认摄像头
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    cap = cv2.VideoCapture(1)
    print("camera changed")

print("detection start")

category_count = []

def detect(detection_num):
    for count in range(1,detection_num):
        ret, tmpframe = cap.read()
        if not ret:
            print("failed to read camera")
            break
            
        # 创建副本用于颜色处理
        frame = tmpframe.astype(np.float32) 
        # 增强绿色和红色通道，减少蓝色通道
        frame[:, :, 0] *= 0.92   # B 通道（减少）
        frame[:, :, 1] *= 1.1   # G 通道（增强）
        frame[:, :, 2] *= 1.05  # R 通道（适当增强）    
        # 再整体变暗
        frame *= 0.8
        # 限制值在 0~255 并转回 uint8
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        
        # alpha 是对比度系数，>1 表示增强对比度，beta 是亮度偏移（可选）
        alpha = 1.7  # 对比度增强（1.0 = 原图）
        beta = -60     # 亮度偏移
        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
    
        # 推理
        results = model(frame)
    
        # 获取预测结果（Pandas 格式，方便操作）
        detections = results.pandas().xyxy[0]
    
        # 打印每个识别结果的名称和位置信息
        for index, row in detections.iterrows():          
            name = row['name']
            confidence = row['confidence']
            x1, y1, x2, y2 = map(int, [row['xmin'], row['ymin'], row['xmax'], row['ymax']])
            
            if(abs((x1-x2)*(y1 - y2)) > 6400):
                name = 'elephant'
            
            #print(f"检测到: {name}, 位置: ({x1}, {y1}), ({x2}, {y2}), 置信度: {confidence:.2f}")
            if confidence >= REQUEST_CONFIDENCE:
                # 更新计数数组数据
                attend_flag = 1           
                for it_count in range(len(category_count)):
                    item = category_count[it_count]
                    if(item['category'] == name):
                        category_count[it_count]['count']+=1
                        category_count[it_count]['PX'] = (category_count[it_count]['PX'] + (x1+x2)/2) / category_count[it_count]['count']
                        category_count[it_count]['PY'] = (category_count[it_count]['PY'] + (y1+y2)/2) / category_count[it_count]['count']
                        attend_flag = 0
                        break
                        
                if(attend_flag == 1):
                    category_count.append({
                        "category":name,
                        "PX":(x1+x2)/2,
                        "PY":(y1+y2)/2,
                        "count":1
                    })


    
def sub_callback():
    category_count.clear()
    detect(10)
    
    for res in category_count:
        if(res['count'] >= 1):
            print(f"{res['category']} {int(res['PX'])} {int(res['PY'])}")
    

    
if __name__ == '__main__':
    while(True):
        sub_callback();
    
    cap.release()
    cv2.destroyAllWindows()
