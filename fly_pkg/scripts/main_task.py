#!/usr/bin/env python3

import rospy
from fly_pkg.srv import pos_srv, vision_srv, pos_srvRequest
from fly_pkg.msg import cv_data
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import String

PI = 3.1415926

ON = 1  # Flight mode
Land = -1  # Landing
NO = 0
Ready = 2  # Enter standby mode
ERROR = 3  # Position failure
fly_z = 0.5
vision_open = 0

local_pos = PoseStamped()
aim_pos = pos_srvRequest()

def local_pos_cb(msg):
    global local_pos
    local_pos = msg


current_state = State()
def state_cb(msg):
    global current_state
    current_state = msg

cv_Data = cv_data()
def cv_data_cb(msg):
    global cv_Data
    cv_Data = msg

# def vision_data_server(req):
#     global vision
#     global vision_open
#     vision.x = req.x[0:req.number]
#     rospy.loginfo(f"{vision.x[0]} {vision.x[1]} {vision.x[2]} {req.number}")
#     resp = vision_srv.Response()
#     resp.need_vision = vision_open
#     return resp


def fly_control(pos_x, pos_y, pos_z, yaw, control_flag, angle_accuracy, pos_accuracy):
    global aim_pos
    aim_pos.control_flag = control_flag
    aim_pos.x = pos_x
    aim_pos.y = pos_y
    aim_pos.z = pos_z
    aim_pos.yaw = yaw

    aim_pos.angle_accuracy = angle_accuracy
    aim_pos.pos_accuracy = pos_accuracy


def fly_control_with_time(pos_x, pos_y, pos_z, yaw, control_flag, times, hz, angle_accuracy, pos_accuracy):
    global aim_pos
    global pos_client
    last_time = rospy.Time.now()
    pos_rate = rospy.Rate(hz)
    fly_control(pos_x, pos_y, pos_z, yaw, control_flag, angle_accuracy, pos_accuracy)
    while not rospy.is_shutdown():
        arrive_flag_resq = pos_client.call(aim_pos)
        if control_flag == Land:  # Landing mode
            if not current_state.armed:  # Waiting for mode to switch completely
                break
        else:
            if arrive_flag_resq.arrive_flag == ON:
                if rospy.Time.now() - last_time > rospy.Duration(times):
                    break
            else:
                last_time = rospy.Time.now()

        pos_rate.sleep()


def plane_to_ready():
    global aim_pos
    global pos_client
    pos_rate = rospy.Rate(100)
    aim_pos.control_flag = Ready
    while not rospy.is_shutdown():
        pos_client.call(aim_pos)
        if current_state.mode == "OFFBOARD" and current_state.armed:  # Waiting for mode to switch completely
            break
        
        pos_rate.sleep()


def PID_revise(x_kp, x_ki, x_kd, x_outputMax, x_outputMin, y_kp, y_ki, y_kd, y_outputMax, y_outputMin, z_kp, z_ki, z_kd, z_outputMax, z_outputMin, yaw_kp, yaw_ki, yaw_kd, yaw_outputMax, yaw_outputMin):
    global aim_pos
    aim_pos.x_kp = x_kp
    aim_pos.x_ki = x_ki
    aim_pos.x_kd = x_kd
    aim_pos.x_outputMax = x_outputMax
    aim_pos.x_outputMin = x_outputMin

    aim_pos.y_kp = y_kp
    aim_pos.y_ki = y_ki
    aim_pos.y_kd = y_kd
    aim_pos.y_outputMax = y_outputMax
    aim_pos.y_outputMin = y_outputMin

    aim_pos.z_kp = z_kp
    aim_pos.z_ki = z_ki
    aim_pos.z_kd = z_kd
    aim_pos.z_outputMax = z_outputMax
    aim_pos.z_outputMin = z_outputMin

    aim_pos.yaw_kp = yaw_kp
    aim_pos.yaw_ki = yaw_ki
    aim_pos.yaw_kd = yaw_kd
    aim_pos.yaw_outputMax = yaw_outputMax
    aim_pos.yaw_outputMin = yaw_outputMin
    rospy.loginfo(aim_pos)


def wait_time(times, hz):
    global aim_pos
    global pos_client
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if rospy.Time.now() - last_time > rospy.Duration(times):
            break
        pos_client.call(aim_pos)
        rospy.Rate(hz).sleep()


if __name__ == '__main__':
    rospy.init_node('main_task', anonymous=True)
    pos_client = rospy.ServiceProxy('pos_control', pos_srv)
    local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_pos_cb)
    state_sub = rospy.Subscriber('mavros/state', State, state_cb)
    # vision_task_server = rospy.Service('vision_data', vision_srv, vision_data_server)
    cv_data = rospy.Subscriber('xxx',cv_data,cv_data_cb)
    
    pos_rate = rospy.Rate(100)  # 100hz

    # Unused
    aim_pos.roll = 0
    aim_pos.pitch = 0
    # x_kp, x_ki, x_kd, x_speed_max, x_speed_min = [0.0,0.0,0.0,0.0,0.0]
    # y_kp, y_ki, y_kd, y_speed_max, y_speed_min = [0.0,0.0,0.0,0.0,0.0]
    # z_kp, z_ki, z_kd, z_speed_max, z_speed_min = [0.0,0.0,0.0,0.0,0.0]
    # yaw_kp, yaw_ki, yaw_kd, yaw_speed_max, yaw_speed_min = [0.0,0.0,0.0,0.0,0.0]


    x_kp, x_ki, x_kd, x_speed_max, x_speed_min = rospy.get_param('x_kp'), rospy.get_param('x_ki'), rospy.get_param('x_kd'), rospy.get_param('x_speed_max'), rospy.get_param('x_speed_min')
    y_kp, y_ki, y_kd, y_speed_max, y_speed_min = rospy.get_param('y_kp'), rospy.get_param('y_ki'), rospy.get_param('y_kd'), rospy.get_param('y_speed_max'), rospy.get_param('y_speed_min')
    z_kp, z_ki, z_kd, z_speed_max, z_speed_min = rospy.get_param('z_kp'), rospy.get_param('z_ki'), rospy.get_param('z_kd'), rospy.get_param('z_speed_max'), rospy.get_param('z_speed_min')
    yaw_kp, yaw_ki, yaw_kd, yaw_speed_max, yaw_speed_min = rospy.get_param('yaw_kp'), rospy.get_param('yaw_ki'), rospy.get_param('yaw_kd'), rospy.get_param('yaw_speed_max'), rospy.get_param('yaw_speed_min')

    rospy.loginfo("--------------------------------PID-------------------------------------------")
    rospy.loginfo(f"x_kp = {x_kp} x_ki={x_ki} x_kd={x_kd} x_speed_max={x_speed_max} x_speed_min={x_speed_min}")
    rospy.loginfo(f"y_kp = {y_kp} y_ki={y_ki} y_kd={y_kd} y_speed_max={y_speed_max} y_speed_min={y_speed_min}")
    rospy.loginfo(f"z_kp = {z_kp} z_ki={z_ki} z_kd={z_kd} z_speed_max={z_speed_max} z_speed_min={z_speed_min}")
    rospy.loginfo(f"yaw_kp = {yaw_kp} yaw_ki={yaw_ki} yaw_kd={yaw_kd} yaw_speed_max={yaw_speed_max} yaw_speed_min={yaw_speed_min}")
    rospy.loginfo("--------------------------------PID-------------------------------------------")

    rospy.wait_for_service('pos_control')  # Wait for position server to start
    plane_to_ready()  # Enter standby mode
    # Modify PID parameters
    PID_revise(x_kp, x_ki, x_kd, x_speed_max, x_speed_min, y_kp, y_ki, y_kd, y_speed_max, y_speed_min, z_kp, z_ki, z_kd, z_speed_max, z_speed_min, yaw_kp, yaw_ki, yaw_kd, yaw_speed_max, yaw_speed_min)
    # Square test
    rospy.loginfo("position1")
    fly_control_with_time(0, 0, 1.0, 0 * PI / 180, ON, 0.25, 100, 3 * PI / 180, 0.1)  # x y z yaw model time hz, angle accuracy, position accuracy

    rospy.loginfo("position2")
    fly_control_with_time(0, 1.0, 1.0, 0 * PI / 180, ON, 0.25, 100, 3 * PI / 180, 0.1)  # x y z yaw model time hz, angle accuracy, position accuracy

    rospy.loginfo("position3")
    fly_control_with_time(0, 1.0, 0, 0 * PI / 180, ON, 1, 100, 3 * PI / 180, 0.1)  # x y z yaw model time hz, angle accuracy, position accuracy

    rospy.loginfo("fly_task is close")
    while not rospy.is_shutdown():
        pos_rate.sleep()