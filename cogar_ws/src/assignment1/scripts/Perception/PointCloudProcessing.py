#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range, LaserScan, PointCloud2, Image, PointField
import numpy as np
import struct


class PointCloudProcessingNode:
    """
    PointCloudProcessingNode: A ROS node for processing sensor data into a point cloud.
        
    This node subscribes to various sensor data topics (sonar, LIDAR, RGB, and depth cameras),
    processes the data, and publishes a point cloud representation.

    Topics:
        - Subscribed:
            - /sonar_base (sensor_msgs.msg.Range): Topic providing sonar data.
            - /scan (sensor_msgs.msg.LaserScan): Topic providing LIDAR scan data.
            - /xtion/rgb/image_raw (sensor_msgs.msg.Image): Topic providing RGB image data.
            - /xtion/depth/image_raw (sensor_msgs.msg.Image): Topic providing depth image data.
        - Published:
            - /perception/pointcloud_processing (sensor_msgs.msg.PointCloud2): Topic for publishing processed point cloud data.

    Attributes:
        - pointcloud_pub (rospy.Publisher): Publisher for processed point cloud data.
        - sonar_data (sensor_msgs.msg.Range): Stores the sonar data received from the subscriber.
        - lidar_data (sensor_msgs.msg.LaserScan): Stores the LIDAR scan data received from the subscriber.
        - rgb_data (sensor_msgs.msg.Image): Stores the RGB image data received from the subscriber.
        - depth_data (sensor_msgs.msg.Image): Stores the depth image data received from the subscriber.
        - rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
        - __init__(): Initializes the node, sets up subscribers and publishers, and starts the main loop.
        - sonar_callback(msg): Callback function for the sonar data subscriber.
        - lidar_callback(msg): Callback function for the LIDAR scan data subscriber.
        - rgb_callback(msg): Callback function for the RGB image data subscriber.
        - depth_callback(msg): Callback function for the depth image data subscriber.
        - ready_to_process(): Checks if the node has received all necessary sensor data.
        - create_dummy_pointcloud(): Creates a dummy PointCloud2 message with random points.
        - process_pointcloud(): Processes the sensor data into a point cloud and publishes it.
    """
    def __init__(self):
        rospy.init_node('pointcloud_processing_node')

        rospy.loginfo("Point Cloud Processing Node initialized.")

        # Subscribers
        rospy.Subscriber('/sonar_base', Range, self.sonar_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
       # Subscribers to RGB and Depth camera
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)

        # Publisher
        self.pointcloud_pub = rospy.Publisher('/perception/pointcloud_processing', PointCloud2, queue_size=10)

        # Internal storage
        self.sonar_data = None
        self.lidar_data = None
        self.rgb_data = None
        self.depth_data = None

        self.rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown():
            if self.ready_to_process():
                self.process_pointcloud()
            self.rate.sleep()

    def sonar_callback(self, msg):
        rospy.loginfo("Received sonar data.")
        self.sonar_data = msg

    def lidar_callback(self, msg):
        rospy.loginfo("Received LIDAR scan.")
        self.lidar_data = msg

    def rgb_callback(self, msg):
        rospy.loginfo("Received processed image data.")
        self.image_data = msg.data

    def depth_callback(self, msg):
        rospy.loginfo("Received depth image data.")
        self.depth_data = msg.data

    def ready_to_process(self):
        return self.sonar_data is not None and self.lidar_data is not None and self.image_data is not None

    def create_dummy_pointcloud(self):
        """
        Creates a dummy PointCloud2 message with random points.
        """
        # Define the header
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        # Define the fields (x, y, z, intensity)
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]

        # Generate some random points
        num_points = 100
        points = np.random.rand(num_points, 4).astype(np.float32)  # x, y, z, intensity

        # Flatten points into a byte array
        point_data = []
        for point in points:
            point_data.append(struct.pack('ffff', *point))

        point_data = b"".join(point_data)

        # Create the PointCloud2 message
        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.height = 1  # 1-dimensional array of points
        pointcloud.width = num_points
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16  # 4 fields x 4 bytes/field
        pointcloud.row_step = pointcloud.point_step * num_points
        pointcloud.is_dense = True  # No invalid points
        pointcloud.data = point_data

        return pointcloud

    def process_pointcloud(self):
        rospy.loginfo("Processing data into a point cloud...")
        # Dummy processing result
        dummy_pointcloud = self.create_dummy_pointcloud()
        self.pointcloud_pub.publish(dummy_pointcloud)
        rospy.loginfo("Published dummy point cloud.")

if __name__ == '__main__':
    try:
        PointCloudProcessingNode()
    except rospy.ROSInterruptException:
        pass
