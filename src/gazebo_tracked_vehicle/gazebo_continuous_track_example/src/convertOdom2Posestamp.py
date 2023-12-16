#!/usr/bin/env python  
from cmath import pi
import roslib
import rospy
from geometry_msgs import msg
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped

def Position_callback(odom_data):
    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "world"

    goal.pose.position.x = odom_data.pose.pose.position.x
    goal.pose.position.y = odom_data.pose.pose.position.y
    goal.pose.position.z = odom_data.pose.pose.position.z

    goal.pose.orientation.x = odom_data.pose.pose.orientation.x
    goal.pose.orientation.y = odom_data.pose.pose.orientation.y
    goal.pose.orientation.z = odom_data.pose.pose.orientation.z
    goal.pose.orientation.w = odom_data.pose.pose.orientation.w

    goal_publisher.publish(goal)


if __name__ == '__main__':
    rospy.init_node('convertOdom2Posestamp')

    rospy.Subscriber("/ground_truth/camera_state",Odometry,Position_callback) 
    goal_publisher = rospy.Publisher("/ground_truth/camera_posestamp", PoseStamped, queue_size=1)

    rospy.spin()