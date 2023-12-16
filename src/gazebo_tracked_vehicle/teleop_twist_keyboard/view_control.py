#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import PyKDL
import math
from math import pi
from math import floor
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion,quaternion_from_euler 
import sys, select, termios, tty

gimbal_vel_fb = [0,0,0]
gimbal_pos_fb = [0,0,0]

body_fdb_imu_pos = [0,0,0]
gimbal_fdb_imu_pos = [0,0,0]
gimbal_fdb_imu_vel = [0,0,0]
gimbal_des = [0,0,0]
gimbal_vec_ref = [0,0,0]
gimbal_vec_ctr = [0,0,0]

Follow_Mode = True
# i_clamp_max = 0.5
# out_clamp_max = 1000
Iout = 0

# ref feedback; set 
def PID_calc(Kp, Ki, fed_val, set_val, i_clamp_max, out_clamp_max):
    error = set_val - fed_val
    Pout = Kp * error
    global Iout
    Iout = Iout + Ki * error
    if(Iout > i_clamp_max):
        Iout = i_clamp_max
    elif(Iout < -i_clamp_max):
        Iout = -i_clamp_max
    out = Pout + Iout
    if(out > out_clamp_max):
        out = out_clamp_max
    elif(out < -out_clamp_max):
        out = -out_clamp_max
    return out

def callback(data): 
    # 0: pitch 1: yaw
    for i in range(2):
        gimbal_vel_fb[i] = data.velocity[i]
        gimbal_pos_fb[i] = data.position[i]
    
    # gimbal_pos[0] = gimbal_pos[0] + 0.77      # 此参数影响ORBslam z轴漂移
    print("gimbal position:",  gimbal_pos_fb)
    # print("gimbal velocity:",  gimbal_vel)
    # print(gimbal_pos[1])
    

def callback_pitch(data):
    print("pitch desire angle: %f \r" % (data.data))

    gimbal_pitch_des = data.data

    if gimbal_pitch_des < -math.pi/6:
      gimbal_pitch_des = -math.pi/6
    elif gimbal_pitch_des > math.pi/6:
      gimbal_pitch_des = math.pi/6

    print("callback_pitch gimbal position:",  gimbal_pos_fb[0])

    gimbal_pitch_vel = PID_calc(10, 0, gimbal_pos_fb[0], gimbal_pitch_des, 0.2, 0.2)

    print("gimbal pitch cmd:",  gimbal_pitch_vel)
    pub_p.publish(gimbal_pitch_vel)

def callback_yaw(data):
    print("yaw desire angle: %f \r" % (data.data))

    gimbal_yaw_des = data.data

    if gimbal_yaw_des < -math.pi/2:
      gimbal_yaw_des = -math.pi/2
    elif gimbal_yaw_des > math.pi/2:
      gimbal_yaw_des = math.pi/2

    print("callback_pitch gimbal position:",  gimbal_pos_fb[1])

    gimbal_yaw_vel = PID_calc(10, 0, gimbal_pos_fb[1], gimbal_yaw_des, 0.2, 0.6)

    print("gimbal yaw cmd:",  gimbal_yaw_vel)
    pub_y.publish(gimbal_yaw_vel)


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    rospy.init_node('control')
    settings = termios.tcgetattr(sys.stdin)

    pub_p = rospy.Publisher('/joint10_velocity_controller/command',Float64, queue_size=3)
    pub_y = rospy.Publisher('/joint11_velocity_controller/command',Float64, queue_size=3)

    rospy.Subscriber("/pitch_position_controller", Float64, callback_pitch)
    rospy.Subscriber("/yaw_position_controller", Float64, callback_yaw)
    rospy.Subscriber("/joint_states",JointState,callback)

    init_flag = 0

    time.sleep(0.1)
    while(1):
        stop_key = getKey()

        if stop_key == '\x03':
            print "exit"
            break

