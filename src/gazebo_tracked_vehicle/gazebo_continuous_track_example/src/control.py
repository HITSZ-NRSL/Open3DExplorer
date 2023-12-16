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

velocity=[0,0,0,0,0,0]
position=[0,0,0,0,0,0]
gimbal_vec = [0,0,0]
gimbal_pos = [0,0,0]
body_fdb_imu_pos = [0,0,0]
gimbal_fdb_imu_pos = [0,0,0]
gimbal_fdb_imu_vel = [0,0,0]
gimbal_des = [0,0,0]
gimbal_vec_ref = [0,0,0]
gimbal_vec_ctr = [0,0,0]
k_leg=[0,0,0,0,0,0]     # 反馈腿转动的圈数，运动一圈为1，
v=Twist()
state=[0,0,0,0,0,0,0,0,0]
# break_value = 0
train_state=Float64MultiArray()

break_=Int16()
break_.data=0

train_state.data=state

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

# 反馈顺序：pitch->roll->yaw
# 控制pub顺序：roll->pitch->yaw
def callback(data): # 这个什么意思，作用是什么？:反馈腿部位置传感器
    # for i in range(2):
    #     velocity[i] = data.velocity[i]
    #     position[i] = data.position[i]
    #     k_leg[i]=(position[i]+pi)//(2*pi)
    for i in range(2):
        gimbal_vec[i] = data.velocity[i]
        gimbal_pos[i] = data.position[i]
    gimbal_pos[0] = gimbal_pos[0]        # 此参数影响ORBslam z轴漂移
    # print("gimbal position:",  gimbal_pos)
    # print("gimbal velocity:",  gimbal_vec)
    # print(gimbal_pos[1])
    
    # print("gimbal_fdb_imu_pos:", gimbal_fdb_imu_pos) 
    # print("gimbal_des:", gimbal_des) 
    
    # if(Follow_Mode):
    #     gimbal_des[2] = body_fdb_imu_pos[2]
    
    # gimbal_vec_ref[0] = PID_calc(1, 0, gimbal_fdb_imu_pos[1], gimbal_des[1], 0.2, 2)
    # gimbal_vec_ref[1] = PID_calc(1, 0, gimbal_fdb_imu_pos[2], gimbal_des[2], 0.2, 2)

    gimbal_vec_ref[0] = PID_calc(0.6, 0, gimbal_pos[0], gimbal_des[1], 0.2, 0.2)
    gimbal_vec_ref[1] = PID_calc(0.6, 0, gimbal_pos[1], gimbal_des[2], 0.2, 0.6)
    # print(gimbal_vec_ref)
    # print("%.3f" % (gimbal_vec_ref[1]))
    # print('')

    # gimbal_vec_ref[0] = PID_calc(20, 2, gimbal_pos[0], gimbal_des[0])      # roll position feedback
    # gimbal_vec_ref[1] = PID_calc(20, 2, gimbal_pos[1], gimbal_des[1])

    # pub_r.publish(gimbal_vec_ctr[0])
    # pub_p.publish(0)
    # pub_y.publish(0)
    pub_p.publish(gimbal_vec_ref[0])
    pub_y.publish(gimbal_vec_ref[1])

# return rpy
# def callback_d435imu(data):
#     rot = PyKDL.Rotation.Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
#     gimbal_fdb_imu_pos[0] = rot.GetRPY()[0]
#     gimbal_fdb_imu_pos[1] = rot.GetRPY()[1]
#     gimbal_fdb_imu_pos[2] = -rot.GetRPY()[2]
    # print("callback_imu:", gimbal_fdb_imu_pos)
    # gimbal_fdb_imu_vel[0] = data.angular_velocity.x     # roll
    # gimbal_fdb_imu_vel[1] = data.angular_velocity.y     # pitch
    # gimbal_fdb_imu_vel[2] = data.angular_velocity.z     # yaw
    # return rot.GetRPY()

# return rpy
# def callback_bodyimu(data):
#     rot = PyKDL.Rotation.Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
#     body_fdb_imu_pos[0] = rot.GetRPY()[0]
#     body_fdb_imu_pos[1] = rot.GetRPY()[1]
#     body_fdb_imu_pos[2] = -rot.GetRPY()[2]
    # return rot.GetRPY()

# data.x->roll
# data.y->pitch
# data.z->yaw
def callback_bestview(data):
    print("data: %f %f %f \r" % (data.x, data.y, data.z))

    gimbal_des[0] = data.x
    gimbal_des[1] = -data.y
    gimbal_des[2] = data.z

    if gimbal_des[1] < -math.pi/6:
      gimbal_des[1] = -math.pi/6
    elif gimbal_des[1] > math.pi/6:
      gimbal_des[1] = math.pi/6

    # if gimbal_des[2] < -math.pi/2:
    #   gimbal_des[2] = -math.pi/2
    # elif gimbal_des[2] > math.pi/2:
    #   gimbal_des[2] = math.pi/2


    # if (data.z < 0) or ((data.z > 0) and (data.z < math.pi/2)):
    #   gimbal_des[2] = data.z + math.pi/2
    # else:
    #   gimbal_des[2] = data.z - (math.pi*3/2)

    # gimbal_des[0] = 0
    # gimbal_des[1] = 0
    # gimbal_des[2] = -3.14
    print("gimbal_des: %f %f %f \r" % (gimbal_des[0], gimbal_des[1], gimbal_des[2]))

def callback2(data):
    v.linear = data.linear
    v.angular = data.angular

def callback_break(data):
    break_.data = data.data


def callback3(data):
    state[0]=data.pose[1].position.x
    state[1]=data.pose[1].position.y
    state[2]=data.pose[1].position.z
    state[3]=data.pose[1].orientation.x
    state[4]=data.pose[1].orientation.y
    state[5]=data.pose[1].orientation.z
    state[6]=data.pose[1].orientation.w
    # train_state.data.append(state)
    #print(state)
    
def set(velocity_d,position_d):
    is_ok = [0,0,0,0,0,0]
    # print("velocity_d:",  velocity_d)
    # print("gimbal control:",  gimbal_vec_ref)
    while(1):
        # pub1.publish(position_d[0]-position[0])
        # if(abs(position_d[0]-position[0]) < pi/200):
        #     pub1.publish(0)
        #     is_ok[0] = 1

        if(abs(position_d[0]-position[0]) > pi/15):
                if(position_d[0] > position[0]):
                    pub1.publish(velocity_d[0])
                else:
                    pub1.publish(-velocity_d[0])
        elif(abs(position_d[0]-position[0]) > pi/200): # 位置控制精度pi/200
                if(position_d[0] > position[0]):
                    pub1.publish(0.4)
                else:
                    pub1.publish(-0.4)
        else:    
            pub1.publish(0)
            is_ok[0] = 1

        if(abs(position_d[1]-position[1])>pi/15):
                if(position_d[1]>position[1]):
                    pub2.publish(velocity_d[1])
                else:
                    pub2.publish(-velocity_d[1])
        elif(abs(position_d[1]-position[1])>pi/200):
                if(position_d[1]>position[1]):
                    pub2.publish(0.4)
                else:
                    pub2.publish(-0.4)                   
        else:
            pub2.publish(0)
            is_ok[1]=1

        if(abs(position_d[2]-position[2])>pi/15):
                if(position_d[2]>position[2]):
                    pub3.publish(velocity_d[2])
                else:
                    pub3.publish(-velocity_d[2])
        elif(abs(position_d[2]-position[2])>pi/200):
                if(position_d[2]>position[2]):
                    pub3.publish(0.4)
                else:
                    pub3.publish(-0.4)   
        else:
            pub3.publish(0)
            is_ok[2]=1

        if(abs(position_d[3]-position[3])>pi/15):
                if(position_d[3]>position[3]):
                    pub4.publish(velocity_d[3])
                else:
                    pub4.publish(-velocity_d[3])
        elif(abs(position_d[3]-position[3])>pi/200):
                if(position_d[3]>position[3]):
                    pub4.publish(0.4)
                else:
                    pub4.publish(-0.4)   
        else:
            pub4.publish(0)
            is_ok[3]=1

        if(abs(position_d[4]-position[4])>pi/15):
                if(position_d[4]>position[4]):
                    pub5.publish(velocity_d[4])
                else:
                    pub5.publish(-velocity_d[4])
        elif(abs(position_d[4]-position[4])>pi/200):
                if(position_d[4]>position[4]):
                    pub5.publish(0.4)
                else:
                    pub5.publish(-0.4)   
        else:
            pub5.publish(0)
            is_ok[4]=1

        if(abs(position_d[5]-position[5])>pi/15):
                if(position_d[5]>position[5]):
                    pub6.publish(velocity_d[5])
                else:
                    pub6.publish(-velocity_d[5])
        elif(abs(position_d[5]-position[5])>pi/200):
                if(position_d[5]>position[5]):
                    pub6.publish(0.4)
                else:
                    pub6.publish(-0.4)   
        else:
            pub6.publish(0)
            is_ok[5]=1

        if(is_ok==[1,1,1,1,1,1]):
            break

def keep_static():
    set([0,0,0,0,0,0],[0,0,0,0,0,0])
    time.sleep(0.02)



def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# def turn_right():
#     k=k_leg[:]
#     v1=[pi/9,pi*17/9,pi/9,pi*17/9,pi/9,pi*17/9]
#     p1=[-pi/9+2*k[0]*pi,-pi*17/9+2*k[1]*pi,-pi/9+2*k[2]*pi,pi*17/9+2*k[3]*pi,pi/9+2*k[4]*pi,pi*17/9+2*k[5]*pi]
#     time.sleep(0.1)
#     set(v1,p1)
#     time.sleep(0.1)
#     for i in range(1):
#         # v2=[pi/9,pi*8/9,pi/9,pi*8/9,pi/9,pi*8/9]
#         v2=[pi*16/9,pi*2/9,pi*16/9,pi*2/9,pi*16/9,pi*2/9]
#         p2=[pi*(-17.0/9+2*k[0]),pi*(-19.0/9+2*k[1]),pi*(-17.0/9+2*k[2]),pi*(19.0/9+2*k[3]),pi*(17.0/9+2*k[4]),pi*(19.0/9+2*k[5])]
#         set(v2,p2)
#         time.sleep(0.1)
#         # v2=[pi*8/9,pi/9,pi*8/9,pi/9,pi*8/9,pi/9]
#         v2=[pi*2/9,pi*16/9,pi*2/9,pi*16/9,pi*2/9,pi*16/9]
#         p2=[pi*(-19.0/9+2*k[0]),pi*(-35.0/9+2*k[1]),pi*(-19.0/9+2*k[2]),pi*(35.0/9+2*k[3]),pi*(19.0/9+2*k[4]),pi*(35.0/9+2*k[5])]
#         set(v2,p2)
#         time.sleep(0.1)
#         for i in range(3):
#             k[i]=k[i]-1
#             k[i+3]=k[i+3]+1
#     v1=[pi/9,pi*17/9,pi/9,pi*17/9,pi/9,pi*17/9]
#     p1=[2*k[0]*pi,2*k[1]*pi,2*k[2]*pi,2*k[3]*pi,2*k[4]*pi,2*k[5]*pi]
#     time.sleep(0.1)
#     set(v1,p1)
#     time.sleep(0.1)
#     print(k)

if __name__=="__main__":
    rospy.init_node('control')
    settings = termios.tcgetattr(sys.stdin)

    # rospy.Subscriber("/cmd_vel",Twist,callback)
    # pub1 = rospy.Publisher('/amphi_robot/joint1_velocity_controller/command',Float64, queue_size=3)
    # pub2 = rospy.Publisher('/amphi_robot/joint2_velocity_controller/command',Float64, queue_size=3)
    # pub3 = rospy.Publisher('/amphi_robot/joint3_velocity_controller/command',Float64, queue_size=3)
    # pub4 = rospy.Publisher('/amphi_robot/joint4_velocity_controller/command',Float64, queue_size=3)
    # pub5 = rospy.Publisher('/amphi_robot/joint5_velocity_controller/command',Float64, queue_size=3)
    # pub6 = rospy.Publisher('/amphi_robot/joint6_velocity_controller/command',Float64, queue_size=3)
    # pub_r = rospy.Publisher('/amphi_robot/joint7_velocity_controller/command',Float64, queue_size=3)
    pub_p = rospy.Publisher('/joint10_velocity_controller/command',Float64, queue_size=3)
    pub_y = rospy.Publisher('/joint11_velocity_controller/command',Float64, queue_size=3)

    # rospy.Subscriber("/body_imu", Imu, callback_bodyimu)
    # rospy.Subscriber("/d435i/imu", Imu, callback_d435imu)
    rospy.Subscriber("/best_view_robot", Vector3, callback_bestview)
    rospy.Subscriber("/joint_states",JointState,callback)
    # rospy.Subscriber("/cmd_vel",Twist,callback2)
    # rospy.Subscriber("/gazebo/model_states",ModelStates,callback3)
    # pub7 = rospy.Publisher('/is_done',Int16, queue_size=3)
    # pub8 = rospy.Publisher('/data',Float64MultiArray,queue_size=3)
    # rospy.Subscriber("break_", Int16, callback_break)

    init_flag = 0

    time.sleep(0.1)
    while(1):
        # pub_r.publish(0)
        # pub_p.publish(0)
        # pub_y.publish(0)
        # if (v.linear.x == 0.0) and (v.angular.z == 0.0) and init_flag == 0:
        #     # print("keep static1")
        #     keep_static()
        #     # print("keep static2")

        stop_key = getKey()

        if stop_key == '\x03':
            print "exit"
            # pub_break.publish(1)
            # time.sleep(1)
            break
        # if(v.linear.x==1.0):
        #     init_flag = 1
        #     #把训练数据：位姿+线速度+角速度 包装成训练信息
        #     train_state.data[7]=1
        #     train_state.data[8]=0
        #     pub8.publish(train_state)
        #     #往前运动
        #     move_forward()
        #     #发送运动结束指令
        #     pub7.publish(1)
        #     v.linear.x=0
        #     v.angular.z=0
        # if(v.angular.z==-1):
        #     init_flag = 1
        #     train_state.data[7]=0
        #     train_state.data[8]=-1
        #     pub8.publish(train_state)
        #     turn_left()
        #     pub7.publish(1)
        #     v.linear.x=0
        #     v.angular.z=0
        # if(v.angular.z==1):
        #     init_flag = 1
        #     train_state.data[7]=0
        #     train_state.data[8]=1
        #     pub8.publish(train_state)
        #     turn_right()
        #     pub7.publish(1)
        #     v.linear.x=0
        #     v.angular.z=0
        # if(v.linear.x==-1):
        #     init_flag = 1
        #     move_backward()
        #     pub7.publish(1)
        #     v.linear.x=0
        #     v.angular.z=0
    	# if(v.linear.x==2.0):
        #     init_flag = 1
        #     six_forward()
        #     pub7.publish(1)
        #     v.linear.x=0
        #     v.angular.z=0
        # if break_.data == 1:
        #     init_flag = 1
        #     print "exit"
        #     break

        # pub7.publish(1)
        
    
    # print(k_leg)
    # v1=[pi*17/18,pi/18,pi*17/18,pi/18,pi*17/18,pi/18]
    # v1=[pi*17/9,pi/9,pi*17/9,pi/9,pi*17/9,pi/9]
    # p1=[pi*17/9+2*k*pi,pi/9+2*k*pi,pi*17/9+2*k*pi,pi/9+2*k*pi,pi*17/9+2*k*pi,pi/9+2*k*pi]
    # time.sleep(1)
    # set(v1,p1)
    # time.sleep(1)
    
    
    # while(1):
    #     # v2=[pi/9,pi*8/9,pi/9,pi*8/9,pi/9,pi*8/9]
    #     v2=[pi*2/9,pi*16/9,pi*2/9,pi*16/9,pi*2/9,pi*16/9]
    #     p2=[pi*(19.0/9+2*k),pi*(17.0/9+2*k),pi*(19.0/9+2*k),pi*(17.0/9+2*k),pi*(19.0/9+2*k),pi*(17.0/9+2*k)]
    #     set(v2,p2)
    #     time.sleep(0.1)
    #     # v2=[pi*8/9,pi/9,pi*8/9,pi/9,pi*8/9,pi/9]
    #     v2=[pi*16/9,pi*2/9,pi*16/9,pi*2/9,pi*16/9,pi*2/9]
    #     p2=[pi*(35.0/9+2*k),pi*(19.0/9+2*k),pi*(35.0/9+2*k),pi*(19.0/9+2*k),pi*(35.0/9+2*k),pi*(19.0/9+2*k)]
    #     set(v2,p2)
    #     time.sleep(0.1)
    #     k=k+1


    # time.sleep(2)
    # print("ssdssd")
    


