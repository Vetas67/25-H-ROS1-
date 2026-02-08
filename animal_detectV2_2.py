#!/usr/bin/env python3
import cv2
import numpy as np
import os
import rospy
from animal_detector.msg import animal_detection_info
from animal_detector.msg import animal_detection_cmd

# === 参数区（你可以根据实际情况调节） ===
ORB_FEATURES = 2000
MATCH_DIST_THRESHOLD = 50
LOWE_RATIO = 0.8
MIN_GOOD_MATCHES = 10
MIN_INLIERS = 8
SHOW_WINDOW = False  # 设置为 False 用于无头设备

# === 初始化 ORB 特征提取器 ===
orb = cv2.ORB_create(nfeatures=ORB_FEATURES)
bf = cv2.BFMatcher(cv2.NORM_HAMMING)

# === 加载参考图片和特征 ===
print("加载参考图片...")
reference_images = []
ref_dir = "./reference_images"

for category in os.listdir(ref_dir):
    category_path = os.path.join(ref_dir, category)
    if not os.path.isdir(category_path):
        continue
    for fname in os.listdir(category_path):
        img_path = os.path.join(category_path, fname)
        img = cv2.imread(img_path, 0)
        if img is None:
            continue
        kp, des = orb.detectAndCompute(img, None)
        if des is None or len(kp) < 5:
            print(f"[跳过] {fname} 无足够特征点")
            continue
        reference_images.append({
            "category": category,
            "name": fname,
            "image": img,
            "keypoints": kp,
            "descriptors": des
        })

print(f"共加载参考图像：{len(reference_images)} 张\n")

# 工具函数，判断是否近似矩形
def is_rectangle(pts, angle_thresh=(60, 120)):
    pts = pts.reshape(4, 2)
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        p3 = pts[(i + 2) % 4]

        v1 = p1 - p2
        v2 = p3 - p2

        # 计算夹角
        angle_rad = np.arccos(np.clip(
            np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6),
            -1.0, 1.0
        ))
        angle_deg = np.degrees(angle_rad)

        if not (angle_thresh[0] <= angle_deg <= angle_thresh[1]):
            return False
    return True

# === 初始化摄像头 ===
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    cap = cv2.VideoCapture(1)
    print("failed to open camera 0,change to camera 1")


category_count = []

# 检测函数，参数是一轮检测的次数
def detect(detection_num):
    # 检测多次    
    for count in range(1,detection_num):
        ret, frame = cap.read()
        if not ret:
            break
    
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        kp_frame, des_frame = orb.detectAndCompute(gray, None)
        if des_frame is None:
            continue
        # 遍历每个图片，看看它们是否出现在了图中
        for ref in reference_images:
            des_ref = ref["descriptors"]
            kp_ref = ref["keypoints"]
    
            # 匹配
            matches = bf.knnMatch(des_ref, des_frame, k=2)
            good = []
            for m_n in matches:
                if len(m_n) < 2:
                    continue
                m, n = m_n
                if m.distance < LOWE_RATIO * n.distance and m.distance < MATCH_DIST_THRESHOLD:
                    good.append(m)

    
            if len(good) < MIN_GOOD_MATCHES:
                continue
    
            # 获取点集
            src_pts = np.float32([kp_ref[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
    
            # Homography + RANSAC
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is None or mask is None or mask.ravel().tolist().count(1) < MIN_INLIERS:
                continue
    
            # 变换参考图角点
            h, w = ref["image"].shape
            corners = np.float32([[0,0], [0,h], [w,h], [w,0]]).reshape(-1, 1, 2)
            projected = cv2.perspectiveTransform(corners, M)
            
            # 判断是否为矩形形状,跳过非矩形匹配
            if not is_rectangle(projected):
                continue  
            
            # 到这里才算成功匹配
            cx = int(np.mean(projected[:, 0, 0]))
            cy = int(np.mean(projected[:, 0, 1]))
            
            # 更新计数数组数据
            rospy.loginfo("find")
            attend_flag = 1           
            for it_count in range(len(category_count)):
                item = category_count[it_count]
                if((item['category'] == ref['category']) and ((item['PX'] - cx)*(item['PX'] - cx) + (item['PY'] - cy)*(item['PY'] - cy) < 20000)):
                    category_count[it_count]['count']+=1
                    category_count[it_count]['PX'] = (category_count[it_count]['PX'] + cx)/category_count[it_count]['count']
                    category_count[it_count]['PY'] = (category_count[it_count]['PY'] + cy)/category_count[it_count]['count']
                    attend_flag = 0
                    break
                    
            if(attend_flag == 1):
                category_count.append({
                    "category":ref['category'],
                    "PX":cx,
                    "PY":cy,
                    "count":1
                })
            
            
      
    
# ROS配置代码
rospy.init_node('animal_detector__node', anonymous=True)  
pub = rospy.Publisher('animal_detection_info', animal_detection_info, queue_size=10)

def publish(Category, PX, PY):
    msg = animal_detection_info()
    msg.Category = Category
    msg.PositionX = PX
    msg.PositionY = PY
    pub.publish(msg)
    
def sub_callback(cmd_msg):
    rospy.loginfo("detection start")
    category_count.clear()
    detect(5)
    for res in category_count:
        if(res['count'] >= 1):
            rospy.loginfo(f"{res['category']}")
            publish(res['category'], int(res['PX']), int(res['PY']))
    
sub = rospy.Subscriber('topic', animal_detection_cmd, sub_callback)
rospy.loginfo("ROS:animal_detection start")
    
    
if __name__ == '__main__':
    rospy.spin()
    
    cap.release()
    cv2.destroyAllWindows()