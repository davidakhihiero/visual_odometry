#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
import numpy as np
import open3d as o3d
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion

last_cloud = None
last_pose = None
initial_pose = np.eye(4)
pose_pub = None

def cloud_callback(msg):
    global last_cloud, last_pose

    T = None
    if last_cloud == None:
        last_pose = np.array(initial_pose)
        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        pc_array = np.array(list(pc))

        pc_points = np.zeros((len(pc_array), 3), dtype=np.float64)
        for i, p in enumerate(pc_array):
            pc_points[i, 0] = p[0]  # x
            pc_points[i, 1] = p[1]  # y
            pc_points[i, 2] = p[2]  # z

        last_cloud = o3d.geometry.PointCloud()
        last_cloud.points = o3d.utility.Vector3dVector(pc_points)
        last_cloud = current_cloud.voxel_down_sample(voxel_size=0.05)  
    else:
        pc = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        pc_array = np.array(list(pc))

        pc_points = np.zeros((len(pc_array), 3), dtype=np.float64)
        for i, p in enumerate(pc_array):
            pc_points[i, 0] = p[0]  # x
            pc_points[i, 1] = p[1]  # y
            pc_points[i, 2] = p[2]  # z

        current_cloud = o3d.geometry.PointCloud()
        current_cloud.points = o3d.utility.Vector3dVector(pc_points)
        current_cloud = current_cloud.voxel_down_sample(voxel_size=0.05)  

        # Perform registration using ICP
        threshold = 0.05
        T = o3d.pipelines.registration.registration_icp(
            last_cloud, current_cloud, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint()
        )

        last_pose = np.dot(T.transformation, last_pose)
        # print(T.transformation)
        last_cloud = current_cloud

    
    pose_msg = PoseStamped()

    R = np.eye(4)
    R[:3,:3] = last_pose[:3,:3]
    
    q = quaternion_from_matrix(R)
        
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "l515_link"
    pose_msg.pose.position.x = last_pose[0, 3]
    pose_msg.pose.position.y = last_pose[1, 3]
    pose_msg.pose.position.z = last_pose[2, 3]
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    pose_pub.publish(pose_msg)


if __name__ == '__main__':
    rospy.init_node('icp_odom')
    pose_pub = rospy.Publisher('/pose_topic', PoseStamped, queue_size=10)
    depth_sub = rospy.Subscriber('/camera/pointcloud', PointCloud2, cloud_callback)
    rospy.spin()

