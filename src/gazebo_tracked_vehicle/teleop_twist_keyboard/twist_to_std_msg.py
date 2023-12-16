#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys, select, termios, tty

msg = ""


class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/sprocket_velocity_controller/command', Float64, queue_size = 10)
        self.publisher_sw = rospy.Publisher('/joint_position_controller/command', Float64, queue_size = 10)

        self.condition = threading.Condition()
        self.done = False
        self.recv_flag = False
        self.start()

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(0.5)
            if self.recv_flag:
                self.recv_flag = False
            else:
                self.publish_msg(0)

    def update(self):
        self.condition.acquire()
        self.recv_flag = True
        self.condition.notify()
        self.condition.release()

    def publish_msg(self, twist):
        msg = Float64()
        msg.data = twist
        self.publisher.publish(msg)
        print("Publish: ", msg.data)

        angle = Float64()
        angle.data = 0.7
        self.publisher_sw.publish(angle)
        print("Publish_sw: ", angle.data)




        
def callback(twist):
    pub_thread.publish_msg(twist.linear.x * 10)
    pub_thread.update()




if __name__=="__main__":
    global publisher, pub_thread
    pub_thread = PublishThread()


    rospy.init_node('twist_to_std_msg')
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rate = rospy.Rate(30)
    
    try:
        rospy.spin()
        # while not rospy.is_shutdown():
        #     rate.sleep()

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()


