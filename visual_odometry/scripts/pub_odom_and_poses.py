#!/usr/bin/env python

import numpy as np
import rospy
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped

pub_l515_att = rospy.Publisher('/vo_att', Vector3Stamped, queue_size=1)
pub_truth_att = rospy.Publisher('/truth_att', Vector3Stamped, queue_size=1)


def visual_odom_callback(odom_msg):
    pose = odom_msg.pose.position
    orientation = odom_msg.pose.orientation


    q = [orientation.x, orientation.y, orientation.z, orientation.w]

    e = euler_from_quaternion(q)

    att_msg = Vector3Stamped()
    att_msg.header.stamp = rospy.Time.now()  
    att_msg.vector.x = e[0] 
    att_msg.vector.y = e[1]  
    att_msg.vector.z = e[2] 

    pub_l515_att.publish(att_msg)


def truth_callback(odom_msg): 
    pose = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation
 
    q = [orientation.x, orientation.y, orientation.z, orientation.w]

    e = euler_from_quaternion(q)

    att_msg = Vector3Stamped()
    att_msg.header.stamp = rospy.Time.now()  
    att_msg.vector.x = e[0] 
    att_msg.vector.y = e[1]  
    att_msg.vector.z = e[2] 

    pub_truth_att.publish(att_msg)

def vicon_callback(transform_msg):
    pose = transform_msg.transform.translation
    orientation = transform_msg.transform.rotation 

    q = [orientation.x, orientation.y, orientation.z, orientation.w]

    e = euler_from_quaternion(q)

    att_msg = Vector3Stamped()
    att_msg.header.stamp = rospy.Time.now()  
    att_msg.vector.x = e[0] 
    att_msg.vector.y = e[1]  
    att_msg.vector.z = e[2] 

    pub_truth_att.publish(att_msg)


if __name__ == '__main__':
    rospy.init_node('publish_odoms_and_poses')

    rospy.wait_for_message('/dji/vicon/dji_m100/dji_m100', TransformStamped)
    # rospy.wait_for_message('/tesse/odom', Odometry)

    rospy.Subscriber('/dji/vicon/dji_m100/dji_m100', TransformStamped, vicon_callback, queue_size=1)
    rospy.Subscriber('/tesse/odom', Odometry, truth_callback, queue_size=1)
    rospy.Subscriber('/pose_topic', PoseStamped, visual_odom_callback, queue_size=1)

    rospy.spin()