#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from assignment1.msg import SensorFusion  # Custom message assumed

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node')

        rospy.loginfo("Sensor Fusion Node initialized.")

        # Subscribers
        rospy.Subscriber('/pointcloud_processing', PointCloud2, self.pointcloud_callback)
        rospy.Subscriber('/image_processing', Image, self.image_callback)
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)

        # Publisher
        self.fused_data_pub = rospy.Publisher('/sensor_fusion', SensorFusion, queue_size=10)

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
