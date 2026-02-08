#!usr/bin/env python3

import serial
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
posmsg = PoseStamped()
fire = Int8()
def communication(msg):
    global posmsg
    posmsg=msg

def led_pose_cb(msg):
    global fire
    fire = msg.data

ser = serial.Serial("/dev/ttyACM1", 115200)
ser.flushInput()  

# 初始化 ROS 节点:命名(唯一)
rospy.init_node("listener_pos")
# 实例化 订阅者 
nowpos = rospy.Subscriber('mavros/local_position/pose', PoseStamped, communication)
led_pose = rospy.Subscriber('led_pos', Int8, led_pose_cb)
rate = rospy.Rate(10)
x=0
y=0
x_area=1
y_area=1
data=[0xff,x_area,x,y_area,y,0,0xaa]
while True:
    weizhi = [posmsg.pose.position.x, posmsg.pose.position.y]
    x=int(posmsg.pose.position.x*100)
    y=int(posmsg.pose.position.y*100)
    print("x",x)
    print("y",y)
    if x>=256:
        x-=256
        x_area=2
    elif x<0:
        x=-x
        x_area=0
    else:
        x_area=1
    if y>=256:
        y-=256
        y_area=2
    elif y<0:
        y=-y
        y_area=0
    else:
        y_area=1
    if fire==0:
        data=[0xff,0xff,x_area,x,y_area,y,0,0xaa]
        print("data",data)
        ser.write(data)
    else:
        data=[0xff,0xff,x_area,x,y_area,y,1,0xaa]
        print("data",data)
        ser.write(data)
        fire=0
    rate.sleep()
