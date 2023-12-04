#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import numpy as np
import open3d as o3d
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion, quaternion_from_euler
import time

last_cloud = None
last_pose = None
initial_pose = np.eye(4)
difference = np.eye(4)
pose_pub = None
last_time = None

last_vel_time = None
last_imu_time = None

orientation = [0, 0, 0]  
position = [0, 0, 0]  
velocity = [0, 0, 0]

gravity = np.array([0.0, 0.0, 0.0])
initialized = False

def cloud_callback(msg):
    global last_cloud, last_pose, last_time, difference, orientation, position, velocity, initialized

    # if last_time is not None and time.time() - last_time < 1:
    #     return
    
    # last_time = time.time()
    if last_time is None:
        last_time = time.time()

    voxel_size = 0.05
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
        initialized = True
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
        current_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30))

        init_transform = get_init_transform()
        threshold = 0.05
        T = o3d.pipelines.registration.registration_icp(
            last_cloud, current_cloud, threshold, np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
           # o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )

        # print(np.linalg.norm(np.dot(np.linalg.inv(T.transformation), init_transform)))
        
        # if np.linalg.norm(np.dot(np.linalg.inv(T.transformation), init_transform)) < 1:
        if last_time is not None and time.time() - last_time > 1:
            last_time = time.time()
            transformation = T.transformation
            orientation = [0, 0, 0]  
            position = [0, 0, 0]
            velocity = [0, 0, 0]
            last_vel_time = None
            last_imu_time = None
        else:
            transformation = np.array(init_transform)
            orientation = [0, 0, 0]  
            position = [0, 0, 0]
            velocity = [0, 0, 0]
            last_vel_time = None
            last_imu_time = None

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


# def transform_callback(msg):
#     global last_pose, difference
#     if last_pose is None:
#         return

#     received_transform = quaternion_matrix([msg.transform.rotation.x,
#                                             msg.transform.rotation.y,
#                                             msg.transform.rotation.z,
#                                             msg.transform.rotation.w])
#     received_transform[:3, 3] = [msg.transform.translation.x,
#                                  msg.transform.translation.y,
#                                  msg.transform.translation.z]

#     difference = np.dot(received_transform, np.linalg.inv(last_pose))


# def velocity_callback(msg):
#     global last_vel_time

#     if last_vel_time is None:
#         dt = 1 / 50
#         last_vel_time = msg.header.stamp.to_sec()
#     else:
#         dt = msg.header.stamp.to_sec() - last_vel_time
#         # if dt < 0.15:
#         #     return
#         last_vel_time = msg.header.stamp.to_sec()

#     # update_position(msg, dt)


def imu_callback(msg):
    global last_imu_time, initialized
    if not initialized:
        return

    if last_imu_time is None:
        dt = 1 / 100
        last_imu_time = msg.header.stamp.to_sec()
    else:
        dt = msg.header.stamp.to_sec() - last_imu_time
        # if dt < 0.1:
        #     return
        last_imu_time = msg.header.stamp.to_sec()

    update_orientation(msg, dt)
    update_position(msg, dt)


# def update_position(velocity_data, dt):
#     global position, velocity
 
#     # position[0] += velocity_data.vector.x * dt
#     # position[1] += velocity_data.vector.y * dt
#     # position[2] += velocity_data.vector.z * dt

#     velocity[0] += velocity_data.linear_acceleration.x * dt
#     velocity[1] += velocity_data.linear_acceleration.y * dt
#     velocity[2] += velocity_data.linear_acceleration.z * dt

#     position[0] += velocity[0] * dt
#     position[1] += velocity[1] * dt
#     position[2] += velocity[2] * dt


def update_position(imu_data, dt):
    global position, velocity, orientation, gravity

    linear_acc = np.array([
        imu_data.linear_acceleration.x,
        imu_data.linear_acceleration.y,
        -imu_data.linear_acceleration.z
    ])

    orientation_quaternion = quaternion_from_euler(
        orientation[0], orientation[1], orientation[2]
    )

    rotation_matrix = quaternion_matrix(orientation_quaternion)[:3, :3]
    global_linear_acc = np.dot(rotation_matrix, linear_acc)

    global_linear_acc_no_gravity = global_linear_acc - gravity

    velocity[0] += global_linear_acc_no_gravity[0] * dt
    velocity[1] += global_linear_acc_no_gravity[1] * dt
    velocity[2] += global_linear_acc_no_gravity[2] * dt

    position[0] += velocity[0] * dt
    position[1] += velocity[1] * dt
    position[2] += velocity[2] * dt


def update_orientation(imu_data, dt):
    global orientation

    roll_rate = imu_data.angular_velocity.x
    pitch_rate = imu_data.angular_velocity.y
    yaw_rate = imu_data.angular_velocity.z

    orientation[0] += roll_rate * dt
    orientation[1] += pitch_rate * dt
    orientation[2] += yaw_rate * dt




def get_init_transform():
    global orientation, position

    quaternion = quaternion_from_euler(orientation[0], orientation[1], orientation[2])

    rotation_matrix = quaternion_matrix(quaternion)[:3, :3]
 
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix  
    transformation_matrix[:3, 3] = [position[0], position[1], position[2]]  

    return transformation_matrix


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
    pc_sub = rospy.Subscriber('/camera/pointcloud', PointCloud2, cloud_callback)
    # rospy.Subscriber('/dji/vicon/dji_m100/dji_m100', TransformStamped, transform_callback)

    # rospy.Subscriber('/dji_sdk/velocity', Vector3Stamped, velocity_callback)
    rospy.Subscriber('/dji_sdk/imu', Imu, imu_callback)
    rospy.spin()

