#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.point_cloud2 as pc2


class RGBDepthToPointcloud:
    def __init__(self):
        rospy.init_node('rgbd_to_pointcloud')

        self.bridge = CvBridge()

        # Subscribe to RGB and Depth topics
        self.rgb_sub = rospy.Subscriber('/l515/color/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/l515/depth/image_rect_raw', Image, self.depth_callback)
        self.rgb_info_sub = rospy.Subscriber('/l515/color/camera_info', CameraInfo, self.rgb_info_callback)
        self.depth_info_sub = rospy.Subscriber('/l515/depth/camera_info', CameraInfo, self.depth_info_callback)

        self.pointcloud_publisher = rospy.Publisher('/camera/pointcloud', PointCloud2, queue_size=1)

        self.rgb_info = None
        self.depth_info = None
        self.depth_image = None

    def rgb_info_callback(self, msg):
        self.rgb_info = msg

    def depth_info_callback(self, msg):
        self.depth_info = msg

    def rgb_callback(self, msg):
        pass

    def depth_callback(self, msg):
        if self.rgb_info is None or self.depth_info is None:
            rospy.logwarn("Camera info not received yet.")
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("Error converting depth image: %s" % str(e))
            return

        fx = self.depth_info.K[0]
        fy = self.depth_info.K[4]
        cx = self.depth_info.K[2]
        cy = self.depth_info.K[5]

        depth_image = np.array(depth_image, dtype=np.float32) / 1000.0  # Convert to meters

        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        z = depth_image[v, u]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Create point cloud
        points = np.vstack((x.flatten(), y.flatten(), z.flatten())).T
        header = msg.header
        pointcloud_msg = pc2.create_cloud_xyz32(header, points)

        # Publish the PointCloud2 message
        self.pointcloud_publisher.publish(pointcloud_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rgb_depth_to_pc = RGBDepthToPointcloud()
        rgb_depth_to_pc.run()
    except rospy.ROSInterruptException:
        pass
