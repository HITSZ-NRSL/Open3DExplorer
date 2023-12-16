#!/usr/bin/env python  
from cmath import pi
import roslib
import rospy
from geometry_msgs import msg
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped

def Position_callback(odom_data):
    goal = PoseWithCovarianceStamped()

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "world"

    goal.pose.pose.position.x = odom_data.pose.pose.position.x
    goal.pose.pose.position.y = odom_data.pose.pose.position.y
    goal.pose.pose.position.z = odom_data.pose.pose.position.z

    goal.pose.pose.orientation.x = odom_data.pose.pose.orientation.x
    goal.pose.pose.orientation.y = odom_data.pose.pose.orientation.y
    goal.pose.pose.orientation.z = odom_data.pose.pose.orientation.z
    goal.pose.pose.orientation.w = odom_data.pose.pose.orientation.w

    goal_publisher.publish(goal)


if __name__ == '__main__':
    rospy.init_node('convertOdom2Posestampwithconv')

    rospy.Subscriber("/ground_truth/camera_state",Odometry,Position_callback) 
    goal_publisher = rospy.Publisher("/ground_truth/camera_posestamp", PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()