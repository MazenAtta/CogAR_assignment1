#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from assignment1.msg import SensorFusion

class SensorFusionNode:
    """
    A ROS node for fusing data from multiple sensors.

    This node subscribes to topics providing data from various sensors (PointCloud, Image, Odometry, IMU), and publishes fused sensor data.

    Topics:
    =======
        - Subscribed:
            - /perception/pointcloud_processing (PointCloud2): Point cloud data from the perception module.
            - /perception/image_processing (Image): Image data from the perception module.
            - /mobile_base_controller/odom (Odometry): Odometry data from the mobile base controller.

        - Published:
            - /perception/sensor_fusion (SensorFusion): Fused sensor data.
    
    Attributes:
    ===========
    Attributes:
        fused_data_pub (rospy.Publisher): Publishes fused sensor data.
        pointcloud_data (PointCloud2): Stores point cloud data.
        image_data (Image): Stores image data.
        odom_data (Odometry): Stores odometry data.
        imu_data (dict): Simulated IMU data.
        rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
    ========
    Methods:
        __init__():
            Initializes the node, sets up subscribers and publishers, and starts the main loop.
        pointcloud_callback(msg):
            Callback function for receiving point cloud data.
        image_callback(msg):
            Callback function for receiving image data.
        odom_callback(msg):
            Callback function for receiving odometry data.
        fake_imu_data():
            Generates simulated IMU data.
        ready_to_fuse():
            Checks if all required sensor data is available for fusion.
        fuse_data():
            Fuses the sensor data and publishes the result.
    """

    def __init__(self):
        rospy.init_node('sensor_fusion_node')

        rospy.loginfo("Sensor Fusion Node initialized.")

        # Subscribers
        rospy.Subscriber('/perception/pointcloud_processing', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/perception/image_processing', Image, self.image_callback)
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

        # Publisher
        self.fused_data_pub = rospy.Publisher('/perception/sensor_fusion', SensorFusion, queue_size=10)

        # Internal data
        self.pointcloud_data = None
        self.image_data = None
        self.odom_data = None
        self.imu_data = self.fake_imu_data()  # Simulated

        self.rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown():
            if self.ready_to_fuse():
                self.fuse_data()
            self.rate.sleep()

    def pointcloud_callback(self, msg):
        rospy.loginfo("Received PointCloud data.")
        self.pointcloud_data = msg

    def image_callback(self, msg):
        rospy.loginfo("Received Image data.")
        self.image_data = msg

    def odom_callback(self, msg):
        rospy.loginfo("Received Odometry data.")
        self.odom_data = msg

    def fake_imu_data(self):
        # Dummy IMU Data
        imu = {
            'orientation': [0, 0, 0, 1],  # Quaternion
            'angular_velocity': [0, 0, 0],
            'linear_acceleration': [0, 0, 0]
        }
        return imu

    def ready_to_fuse(self):
        return self.pointcloud_data is not None and self.image_data is not None and self.odom_data is not None

    def fuse_data(self):
        rospy.loginfo("Fusing sensor data...")

        fused_msg = SensorFusion()

        # Fill in dummy data for FusedData (adjust based on your real FusedData.msg fields)
        fused_msg.header.stamp = rospy.Time.now()
        fused_msg.header.frame_id = "base_link"
        
        # Assume FusedData has these fields (update if needed)
        fused_msg.pointcloud = self.pointcloud_data
        fused_msg.image = self.image_data
        fused_msg.odom = self.odom_data
        fused_msg.imu_orientation = self.imu_data['orientation']
        fused_msg.imu_angular_velocity = self.imu_data['angular_velocity']
        fused_msg.imu_linear_acceleration = self.imu_data['linear_acceleration']

        self.fused_data_pub.publish(fused_msg)
        rospy.loginfo("Published Fused Sensor Data.")

if __name__ == '__main__':
    try:
        SensorFusionNode()
    except rospy.ROSInterruptException:
        pass
