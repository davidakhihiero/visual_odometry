#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import open3d as o3d
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion
import time

last_cloud = None
last_pose = None
initial_pose = np.eye(4)
difference = np.eye(4)
pose_pub = None
last_time = None

def cloud_callback(msg):
    global last_cloud, last_pose, last_time, difference

    if last_time is not None and time.time() - last_time < 0:
        return
    
    last_time = time.time()
    voxel_size = 0.1
    downsample = True
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
        if downsample:
            last_cloud = last_cloud.voxel_down_sample(voxel_size=voxel_size)  
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
        if downsample:
            current_cloud = current_cloud.voxel_down_sample(voxel_size=voxel_size)  

        # Perform registration using ICP
        current_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        threshold = 0.05
        T = o3d.pipelines.registration.registration_icp(
            last_cloud, current_cloud, threshold, difference,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        
        if np.linalg.norm(np.dot(np.linalg.inv(T.transformation), difference)) < 3:
            transformation = T.transformation
        else:
            transformation = np.array(difference)

        last_pose = np.dot(transformation, last_pose)
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


def transform_callback(msg):
    global last_pose, difference
    if last_pose is None:
        return

    received_transform = quaternion_matrix([msg.transform.rotation.x,
                                            msg.transform.rotation.y,
                                            msg.transform.rotation.z,
                                            msg.transform.rotation.w])
    received_transform[:3, 3] = [msg.transform.translation.x,
                                 msg.transform.translation.y,
                                 msg.transform.translation.z]

    difference = np.dot(received_transform, np.linalg.inv(last_pose))


if __name__ == '__main__':
    # translation: 
    #     x: -3.0454221847459277
    #     y: -0.08056010217775714
    #     z: 0.18427528415090083
    # rotation: 
    #     x: 0.008072461353199337
    #     y: -0.010573090757896622
    #     z: -0.36845680178307755
    #     w: 0.9295496922377589

    rospy.init_node('icp_odom')

    initial_q = [0.008072461353199337, -0.010573090757896622, -0.36845680178307755, 0.9295496922377589] # x y z w
    initial_pos = [-3.0454221847459277, -0.08056010217775714, 0.18427528415090083]

    # initial_q = [0.0001110956622209167, 0.0106505768691511022, 0.010428763846818559, 0.99988889070473] # x y z w
    # initial_pos = [-18.36323, 23.76775, 2.505424]

    initial_rot_matrix = quaternion_matrix(initial_q)

    initial_pose[0:3, 0:3] = initial_rot_matrix[0:3, 0:3]
    initial_pose[0, 3] = initial_pos[0]
    initial_pose[1, 3] = initial_pos[1]
    initial_pose[2, 3] = initial_pos[2]

    pose_pub = rospy.Publisher('/pose_topic', PoseStamped, queue_size=1)
    depth_sub = rospy.Subscriber('/camera/pointcloud', PointCloud2, cloud_callback)
    rospy.Subscriber('/dji/vicon/dji_m100/dji_m100', TransformStamped, transform_callback)
    rospy.spin()

