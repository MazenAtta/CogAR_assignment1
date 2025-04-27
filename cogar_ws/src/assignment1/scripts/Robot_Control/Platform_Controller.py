#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from assignment1.msg import SensorFusion

class PlatformControllerNode:
    """
    PlatformControllerNode: A ROS node for controlling the platform's motors based on sensor fusion and path planning data.

    This node subscribes to topics providing sensor fusion data and path setpoints, processes the data,
    and publishes motor commands to control the platform's movement.

    Topics:
    =======
        - Subscribed:
            - /sensor_fusion (custom_msgs.msg.SensorFusion): Topic providing sensor fusion data.
            - /path_planner/setpoint (custom_msgs.msg.Path): Topic providing path setpoints.
        - Published:
            - /motor_driver/left (geometry_msgs.msg.Twist): Topic for publishing left motor commands.
            - /motor_driver/right (geometry_msgs.msg.Twist): Topic for publishing right motor commands.

    Attributes:
    ===========
    Attributes:
        - left_motor_pub (rospy.Publisher): Publisher for the left motor commands.
        - right_motor_pub (rospy.Publisher): Publisher for the right motor commands.
        - current_path (custom_msgs.msg.Path): Stores the latest path setpoint data.
        - current_sensor_data (custom_msgs.msg.SensorFusion): Stores the latest sensor fusion data.
        - rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
    ========
    Methods:
        - __init__(): Initializes the node, sets up subscribers and publishers, and starts the main control loop.
        - sensor_fusion_callback(msg): Callback function for the sensor fusion data subscriber.
        - path_callback(msg): Callback function for the path setpoint data subscriber.
        - control_loop(): Main control loop for processing data and publishing motor commands.

     
    """
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
        self.current_sensor_data = msg
        rospy.loginfo_throttle(2.0, "Received sensor fusion data")

    def path_callback(self, msg):
        self.current_path = msg
        rospy.loginfo("Received new path with %d waypoints", len(msg.poses))

    def control_loop(self):
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
