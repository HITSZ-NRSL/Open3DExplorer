#!/usr/bin/env python  
import roslib
import rospy
from geometry_msgs import msg
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def Position_callback(odom_data):
    curr_time = odom_data.header.stamp

    br = TransformBroadcaster()
    b_to_e = TransformStamped()
    b_to_e.header.stamp = rospy.Time.now()
    b_to_e.header.frame_id = "world"
    b_to_e.child_frame_id = "camera_link"
    b_to_e.transform.translation.x = odom_data.pose.pose.position.x
    b_to_e.transform.translation.y = odom_data.pose.pose.position.y
    b_to_e.transform.translation.z = odom_data.pose.pose.position.z - 0.8
    print('position: ', b_to_e.transform.translation.x, b_to_e.transform.translation.y, b_to_e.transform.translation.z)

    (r, p, y) = tf.transformations.euler_from_quaternion([odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
    print('orinient in Quaternion: ', odom_data.pose.pose.orientation)
    _tmp = r
    r = -p
    p = -_tmp

    q = tf.transformations.quaternion_from_euler(r, p, y)
    print('orinient in Quaternion: ', q)
    print('orinient in RPY: ', r, p, y)

    b_to_e.transform.rotation.x = q[0]
    b_to_e.transform.rotation.y = q[1]
    b_to_e.transform.rotation.z = q[2]
    b_to_e.transform.rotation.w = q[3]

    br.sendTransform(b_to_e)


if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')

    rospy.Subscriber("/ground_truth/camera_state",Odometry,Position_callback) 

    rospy.spin()