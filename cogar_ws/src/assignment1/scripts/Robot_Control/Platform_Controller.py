#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from assignment1.msg import SensorFusion

class PlatformControllerNode:
    def __init__(self):
        rospy.init_node('platform_controller_node')
        rospy.loginfo("Platform Controller Node initialized.")

        # Subscribers
        rospy.Subscriber('/sensor_fusion', SensorFusion, self.sensor_fusion_callback)
        rospy.Subscriber('/path_planner/setpoint', Path, self.path_callback)

        # Publishers for motor drivers
        self.left_motor_pub = rospy.Publisher('/motor_driver/left', Twist, queue_size=10)
        self.right_motor_pub = rospy.Publisher('/motor_driver/right', Twist, queue_size=10)

        # Internal data
        self.current_path = None
        self.current_sensor_data = None
        self.rate = rospy.Rate(10)  # 10 Hz

        # Main loop
        while not rospy.is_shutdown():
            self.control_loop()
            self.rate.sleep()

    def sensor_fusion_callback(self, msg):
        """Process sensor fusion data"""
        self.current_sensor_data = msg
        rospy.loginfo_throttle(2.0, "Received sensor fusion data")

    def path_callback(self, msg):
        """Process path setpoint data"""
        self.current_path = msg
        rospy.loginfo("Received new path with %d waypoints", len(msg.poses))

    def control_loop(self):
        """Simple control loop to drive motors based on path and sensor data"""
        if not self.current_path or not self.current_sensor_data:
            return  # No data yet

        # Create dummy motor commands
        left_cmd = Twist()
        right_cmd = Twist()

        # Populate with dummy values 
        left_cmd.linear.x = 0.2  # Forward velocity
        right_cmd.linear.x = 0.2  # Forward velocity

        # Publish motor commands
        self.left_motor_pub.publish(left_cmd)
        self.right_motor_pub.publish(right_cmd)

if __name__ == '__main__':
    try:
        controller = PlatformControllerNode()
    except rospy.ROSInterruptException:
        pass
