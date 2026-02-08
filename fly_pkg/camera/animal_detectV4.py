#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
import pathlib
# Patch WindowsPath to behave like PosixPath on Linux
if not hasattr(pathlib, 'WindowsPath'):
    from pathlib import PosixPath as WindowsPath
    pathlib.WindowsPath = WindowsPath
else:
    # Linux 上也有这个属性，但不能实例化
    if isinstance(pathlib.Path(), pathlib.WindowsPath):
        pathlib.WindowsPath = pathlib.PosixPath

import cv2
import torch
import rospy
import numpy as np
from animal_detector.msg import animal_detection_info
from animal_detector.msg import animal_detection_cmd

REQUEST_CONFIDENCE = 0.3

# 加载模型（默认使用 yolov5s）
model = torch.hub.load('yolov5', 'custom', path='animal_detector.pt', source='local')
model.eval()

# 打开默认摄像头
cap = cv2.VideoCapture(0)
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
            
            if confidence >= REQUEST_CONFIDENCE:
                rospy.loginfo(f"category: {name} ")
                # 更新计数数组数据
                attend_flag = 1           
                for it_count in range(len(category_count)):
                    item = category_count[it_count]
                    if((item['category'] == name) and ((item['PX'] - (x1+x2)/2)*(item['PX'] - (x1+x2)/2) + (item['PY'] - (y1+y2)/2)*(item['PY'] - (y1+y2)/2) < 5000)):
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


# ROS配置代码
rospy.init_node('animal_detector__node', anonymous=True)  
pub = rospy.Publisher('animal_detection_info', animal_detection_info, queue_size=10)

def publisher(Category, PX, PY, Count):
    msg = animal_detection_info()
    msg.Category = Category
    msg.PositionX = PX
    msg.PositionY = PY
    msg.count = Count
    pub.publish(msg)
    
def sub_callback(cmd_msg):
    rospy.loginfo("detection start")
    category_count.clear()
    detect(10)
    
    detect_count = 0
    Category_Array = []
    PX_Array = []
    PY_Array = []
    
    for res in category_count:
        if(res['count'] >= 1):
            if(res['PX'] > 150 and res['PX'] < 490 and res['PY'] > 100 and res['PY'] < 350):
                detect_count+=1
                rospy.loginfo(f"{res['category']}")
                Category_Array.append(res['category'])
                PX_Array.append(res['PX'])
                PY_Array.append(res['PY'])
            
    publisher(Category_Array, PX_Array, PY_Array, detect_count)
            
    if(detect_count == 0):
        Category_Array.append("NULL")
        PX_Array.append(0)
        PY_Array.append(0)
        publisher(Category_Array, PX_Array, PY_Array, 0)
    
sub = rospy.Subscriber('topic', animal_detection_cmd, sub_callback)
rospy.loginfo("ROS:animal_detection start")

    
if __name__ == '__main__':
    rospy.spin()
    
    cap.release()
    cv2.destroyAllWindows()
