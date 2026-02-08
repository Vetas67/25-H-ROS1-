#!/usr/bin/env python3

import rospy
import math
import tf
from std_msgs.msg import Float32
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Twist
from fly_pkg.srv import pos_srv,pos_srvResponse
from fly_pkg.msg import pos_msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

control_hz = 100
PI = 3.1415926
ON = 1
Land = -1
NO = 0
Ready = 2  # Enter standby mode
ERROR = 3  # Position failure

Land_height = 0  #降落锁电机高度
control_flag = NO  # Control instruction
x_max = [-3.0, 3.0]  # Min and Max value
y_max = [-3.0, 3.0]  # Min and Max value
z_max = 1.0  # Max value


class IMU:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0


mavros_ENU = IMU()


class PID_Obj:
    def __init__(self):
        self.output = 0.0
        self.bias = 0.0
        self.measure = 0.0
        self.last_bias = 0.0
        self.integral = 0.0
        self.last_differential = 0.0
        self.target = 0.0


class PID_Param:
    def __init__(self):
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.differential_filterK = 0.5
        self.outputMin = 0.0
        self.outputMax = 0.0


x_pid_obj = PID_Obj()
x_pid_param = PID_Param()

y_pid_obj = PID_Obj()
y_pid_param = PID_Param()

z_pid_obj = PID_Obj()
z_pid_param = PID_Param()

yaw_pid_obj = PID_Obj()
yaw_pid_param = PID_Param()


def get_q(yaw_aim):
    q = quaternion_from_euler(0, 0, yaw_aim)
    quat = Quaternion()
    quat.x = q[0]
    quat.y = q[1]
    quat.z = q[2]
    quat.w = q[3]
    return quat


def need2turn(nowangle, targetangle):
    need2Turn = targetangle - nowangle
    if need2Turn > PI:
        need2Turn -= 2 * PI
    elif need2Turn < -PI:
        need2Turn += PI
    return need2Turn


current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg


class POS_RESET:
    def __init__(self):
        self.x_bias = 0.0
        self.y_bias = 0.0
        self.z_bias = 0.0
        self.yaw_bias = 0.0


pos_reset = POS_RESET()
last_light_z = 0.0  # Laser break protection


def local_pos_cb(msg):
    global local_pos, mavros_ENU, pos_reset
    local_pos = msg
    
    local_pos.pose.position.x -= pos_reset.x_bias
    local_pos.pose.position.y -= pos_reset.y_bias
    local_pos.pose.position.z -= pos_reset.z_bias

    # Extract quaternion components
    IMU_q = [0.0, 0.0, 0.0, 0.0]
    IMU_q[0] = local_pos.pose.orientation.x
    IMU_q[1] = local_pos.pose.orientation.y
    IMU_q[2] = local_pos.pose.orientation.z
    IMU_q[3] = local_pos.pose.orientation.w

    # Convert quaternion to Euler angles (mavros ENU coordinates)
    euler = euler_from_quaternion(IMU_q)
    mavros_ENU.yaw = euler[2] - PI / 2
    if mavros_ENU.yaw > PI:
        mavros_ENU.yaw -= 2 * PI
    elif mavros_ENU.yaw < -PI:
        mavros_ENU.yaw += 2 * PI

    mavros_ENU.yaw -= pos_reset.yaw_bias
    if mavros_ENU.yaw > PI:
        mavros_ENU.yaw -= 2 * PI
    elif mavros_ENU.yaw < -PI:
        mavros_ENU.yaw += 2 * PI


def debug(flag):
    if flag == 1:
        rospy.loginfo("success")
    else:
        rospy.loginfo("failure")

aim_pos = pos_msg()
def pos_control_server(req):
    global aim_pos, x_pid_param, y_pid_param, z_pid_param, yaw_pid_param, local_pos
    # 位置和控制指令
    global aim_pos_resp
    aim_pos.x = req.x
    aim_pos.y = req.y
    aim_pos.z = req.z
    aim_pos.control_flag = req.control_flag
    # PID参数
    x_pid_param.kp = req.x_kp
    x_pid_param.ki = req.x_ki
    x_pid_param.kd = req.x_kd
    x_pid_param.outputMax = req.x_outputMax
    x_pid_param.outputMin = req.x_outputMin

    # 对于y, z, yaw的PID参数，做类似的处理...
    y_pid_param.kp = req.y_kp
    y_pid_param.ki = req.y_ki
    y_pid_param.kd = req.y_kd
    y_pid_param.outputMax = req.y_outputMax
    y_pid_param.outputMin = req.y_outputMin

    z_pid_param.kp = req.z_kp
    z_pid_param.ki = req.z_ki
    z_pid_param.kd = req.z_kd
    z_pid_param.outputMax = req.z_outputMax
    z_pid_param.outputMin = req.z_outputMin

    yaw_pid_param.kp = req.yaw_kp
    yaw_pid_param.ki = req.yaw_ki
    yaw_pid_param.kd = req.yaw_kd
    yaw_pid_param.outputMax = req.yaw_outputMax
    yaw_pid_param.outputMin = req.yaw_outputMin

    # 精度信息
    aim_pos.angle_accuracy = req.angle_accuracy
    aim_pos.pos_accuracy = req.pos_accuracy

    # 角度信息
    aim_pos.yaw = req.yaw
    if aim_pos.yaw > PI:
        aim_pos.yaw -= 2 * PI
    elif aim_pos.yaw < -PI:
        aim_pos.yaw += 2 * PI

    # 判断是否到达目标位置
    if (abs(local_pos.pose.position.x - aim_pos.x) < aim_pos.pos_accuracy and
            abs(local_pos.pose.position.y - aim_pos.y) < aim_pos.pos_accuracy and
            abs(local_pos.pose.position.z - aim_pos.z) < aim_pos.pos_accuracy):
    
        if abs(need2turn(mavros_ENU.yaw, aim_pos.yaw)) < aim_pos.angle_accuracy:
            aim_pos_resp.arrive_flag = 1  # 到达
            rospy.loginfo("到达目标位置")
        else:
            rospy.loginfo("未到达目标位置")
            aim_pos_resp.arrive_flag = 0  # 未到达
    else:
        aim_pos_resp.arrive_flag = 0  # 未到达
    # rospy.loginfo("x = %f  y = %f  z = %f  yaw = %f", abs(local_pos.pose.position.x - aim_pos.x),
    #              abs(local_pos.pose.position.y - aim_pos.y),
    #              abs(local_pos.pose.position.z - aim_pos.z),
    #              abs(need2turn(mavros_ENU.yaw, aim_pos.yaw)))
    rospy.loginfo(aim_pos_resp.arrive_flag)
    return aim_pos_resp

# 位置式PID
def positional_PID(obj,pid):
    differential = 0
    obj.bias = obj.target - obj.measure

    # 只有误差小 才需要积分项
    if abs(obj.bias) < 0.3:
        # 抗积分饱和
        if obj.output >= pid.outputMax:
            if obj.bias < 0:
                obj.integral += obj.bias / control_hz
        elif obj.output <= pid.outputMin:
            if obj.bias > 0:
                obj.integral += obj.bias / control_hz
        else:
            obj.integral += obj.bias
    else:
        obj.integral = 0

    # 微分项低通滤波
    differential = (obj.bias - obj.last_bias) * pid.differential_filterK + \
                   (1 - pid.differential_filterK) * obj.last_differential

    obj.output = pid.kp * obj.bias + pid.ki * obj.integral + pid.kd * differential

    obj.last_bias = obj.bias
    obj.last_differential = differential

    # 限幅
    if obj.output > pid.outputMax:
        obj.output = pid.outputMax
    elif obj.output < pid.outputMin:
        obj.output = pid.outputMin

    return obj.output

def get_yaw_speed():
    yaw_pid_obj.measure = -need2turn(mavros_ENU.yaw, aim_pos.yaw)
    yaw_pid_obj.target = 0
    return positional_PID(yaw_pid_obj, yaw_pid_param)


if __name__ == '__main__':
    rospy.init_node('pos_task')
    local_pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, local_pos_cb)
    state_sub = rospy.Subscriber('mavros/state', State, state_cb)

    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    main_task_pos_server = rospy.Service('pos_control', pos_srv, pos_control_server)

    aim_pos_resp = pos_srvResponse()
    pose = PoseStamped()
    local_pos = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    aim_pos.control_flag = 0
    aim_pos.yaw = 0.0
    
    vel = Twist()

    rate = rospy.Rate(control_hz)

    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("等待与PX4的连接...")
        rate.sleep()

    while not rospy.is_shutdown() and (local_pos.pose.position.x == 0 and local_pos.pose.position.y == 0 and local_pos.pose.position.z == 0):
        rospy.loginfo("等待位置信息就绪...")
        rate.sleep()
        
    need_reset = 0
    xbias = ybias = zbias = yawbias = 0
    rospy.get_param("need_reset", need_reset)
    for i in range(500):
        local_pos_pub.publish(pose) 
        if(i%25 == 0 and need_reset):
            xbias+= local_pos.pose.position.x
            ybias+= local_pos.pose.position.y
            zbias+= local_pos.pose.position.z
            yawbias+= mavros_ENU.yaw
        
        rate.sleep()

    if (need_reset):
        pos_reset.x_bias = xbias/20
        pos_reset.y_bias = ybias/20 
        pos_reset.z_bias = zbias/20
        pos_reset.yaw_bias = yawbias/20

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    q = get_q(0.0)
    pose.pose.orientation = q

    rospy.get_param("Land_height", Land_height)
    
    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if control_flag != ERROR:
            control_flag = aim_pos.control_flag

        # 根据 control_flag 的值处理不同的飞行控制逻辑
        if control_flag == ERROR:
            vel = Twist()
            vel.linear.z = -0.2
            vel.linear.x = 0
            vel.linear.y = 0
            local_vel_pub.publish(vel)  # 发送速度信息
            rospy.loginfo("ERROR Mode")

        elif control_flag == Ready:
            if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if set_mode_client.call(offb_set_mode).mode_sent == True:
                    rospy.loginfo("Offboard enabled")
                last_request = rospy.Time.now()
            else:
                if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                    if arming_client.call(arm_cmd).success == True:
                        rospy.loginfo("Vehicle armed")
                    last_request = rospy.Time.now()

            local_pos_pub.publish(pose)  # 发送位置信息
            rospy.loginfo("Ready Mode")

        elif control_flag == NO:
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 0
            q = get_q(0.0)
            pose.pose.orientation = q
            local_pos_pub.publish(pose)  # 发送位置信息
            rospy.loginfo("NO Mode - Waiting for main_task")
            

        elif control_flag == ON:
            x_pid_obj.measure = local_pos.pose.position.x
            x_pid_obj.target = aim_pos.x
            y_pid_obj.measure = local_pos.pose.position.y
            y_pid_obj.target = aim_pos.y
            z_pid_obj.measure = local_pos.pose.position.z
            z_pid_obj.target = aim_pos.z

            vel.linear.x = positional_PID(x_pid_obj, x_pid_param)
            vel.linear.y = positional_PID(y_pid_obj, y_pid_param)
            vel.linear.z = positional_PID(z_pid_obj, z_pid_param)

            last_z_val = vel.linear.z

            vel.angular.z = get_yaw_speed()

            local_vel_pub.publish(vel)
            rospy.loginfo("aim_x=%f loc_x=%f aim_y=%f loc_y=%f", aim_pos.x, local_pos.pose.position.x, aim_pos.y, local_pos.pose.position.y)
            rospy.loginfo("main_task is connect----")

        elif control_flag == Land:
            x_pid_obj.measure = local_pos.pose.position.x
            x_pid_obj.target = aim_pos.x
            y_pid_obj.measure = local_pos.pose.position.y
            y_pid_obj.target = aim_pos.y
            z_pid_obj.measure = local_pos.pose.position.z
            z_pid_obj.target = aim_pos.z

            vel.linear.x = positional_PID(x_pid_obj, x_pid_param)
            vel.linear.y = positional_PID(y_pid_obj, y_pid_param)
            vel.linear.z = positional_PID(z_pid_obj, z_pid_param)

            vel.angular.z = get_yaw_speed()

            if local_pos.pose.position.z < Land_height:
                rospy.loginfo("-------land---------")
                if current_state.mode == "OFFBOARD":
                    offb_set_mode.custom_mode = "MANUAL"
                    set_mode_client(offb_set_mode)
                if current_state.armed:
                    arm_cmd.value = False
                    arming_client(arm_cmd)
            else:
                local_vel_pub.publish(vel)
                
        else:
            rospy.loginfo("Unknown control_flag: %d", control_flag)

        rate.sleep()







