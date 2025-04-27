#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from assignment1.msg import SensorFusion

class SLAMNode:
    """
    A ROS node for Simultaneous Localization and Mapping (SLAM).

    This node subscribes to sensor fusion data, processes it to estimate the robot's
    current pose, and publishes the estimated state.

    Topics:
        - Subscribed:
            - /sensor_fusion (SensorFusion): Sensor fusion data containing IMU and odometry information.

        - Published:
            - /slam/estimated_state (PoseStamped): The estimated state of the robot.

    Attributes:
        estimated_state_pub (rospy.Publisher): Publishes the estimated state.
        current_pose (PoseStamped): Stores the current estimated pose.
        rate (rospy.Rate): Controls the loop rate of the node.
    
    Methods:
        __init__():
            Initializes the node, sets up subscribers and publishers, and starts the main loop.
        sensor_fusion_callback(msg):
            Callback function for processing sensor fusion data.
        publish_estimated_state():
            Publishes the current estimated state.
    """

    def __init__(self):
        rospy.init_node('slam_node')
        rospy.loginfo("SLAM Node initialized.")

        # Subscriber
        rospy.Subscriber('/sensor_fusion', SensorFusion, self.sensor_fusion_callback)

        # Publisher
        self.estimated_state_pub = rospy.Publisher('/slam/estimated_state', PoseStamped, queue_size=10)

        # Internal data
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        
        # Set an initial pose (just for testing)
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0

        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Main loop
        while not rospy.is_shutdown():
            # Publish current estimated state
            self.publish_estimated_state()
            self.rate.sleep()

    def sensor_fusion_callback(self, msg):
        rospy.loginfo_throttle(2.0, "Received sensor fusion data")
        
        # Dummy SLAM processing - in a real system, this would implement 
        # localization and mapping algorithms
        
        # Use IMU data for orientation (directly from sensor fusion)
        self.current_pose.pose.orientation.x = msg.imu_orientation[0]
        self.current_pose.pose.orientation.y = msg.imu_orientation[1]
        self.current_pose.pose.orientation.z = msg.imu_orientation[2]
        self.current_pose.pose.orientation.w = msg.imu_orientation[3]
        
        # Use odometry for position (with some random noise to simulate uncertainty)
        # In a real SLAM system, this would combine information from multiple sensors
        self.current_pose.pose.position.x = msg.odom.pose.pose.position.x + np.random.normal(0, 0.01)
        self.current_pose.pose.position.y = msg.odom.pose.pose.position.y + np.random.normal(0, 0.01)
        self.current_pose.pose.position.z = 0.0  # Assuming 2D operation
        
        # Update timestamp
        self.current_pose.header.stamp = rospy.Time.now()

    def publish_estimated_state(self):
        # Update timestamp before publishing
        self.current_pose.header.stamp = rospy.Time.now()
        self.estimated_state_pub.publish(self.current_pose)

if __name__ == '__main__':
    try:
        slam_node = SLAMNode()
    except rospy.ROSInterruptException:
        pass
