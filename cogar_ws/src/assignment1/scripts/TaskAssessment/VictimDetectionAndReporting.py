#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point

class VictimDetectionAndReportingNode:
    """
    A ROS node for detecting and reporting victims based on sensor data.

    This node subscribes to topics providing processed audio, image data, and the robot's current pose.
    It detects victims based on the received data and publishes alerts and victim locations.

    Topics:
    =======
        - Subscribed:
            - /perception/processed_audio (String): Processed audio data from the audio processing node.
            - /slam/estimated_state (PoseStamped): The robot's current pose from the SLAM node.
            - /task_executor/task (String): Task data from the task executor.
            - /perception/image_processing (Image): Processed image data from the image processing node.
        - Published:
            - /victim_detection/alert (String): Alerts for detected victims.
            - /victim_detection/location (Point): Locations of detected victims.

    Attributes:
    ===========
    Attributes:
        current_pose (PoseStamped): The robot's current pose.
        audio_detection (str): The processed audio data received from the subscriber.
        image_data (Image): The processed image data received from the subscriber.
        task_instruction (str): The task instructions received from the subscriber.
        alert_pub (rospy.Publisher): Publishes victim detection alerts.
        victim_location_pub (rospy.Publisher): Publishes victim locations.
        rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
    ========
    Methods:
        __init__():
            Initializes the node, sets up subscribers and publishers, and starts the main loop.
        audio_callback(msg):
            Receives processed audio data.
        estimated_state_callback(msg):
            Receives the robot's current pose.
        task_executor_callback(msg):
            Receives task instruction messages.
        image_processing_callback(msg):
            Receives processed image data.
        ready_for_detection():
            Checks if the node has all the required data to detect victims.
        detect_victims():
            Detects victims based on the received sensor data.
        report_victim(victim_type):
            Reports detected victims by publishing alerts and locations.
    """

    def __init__(self):
        rospy.init_node('victim_detection_and_reporting')
        rospy.loginfo("Victim Detection Node Initialized. Waiting for sensor data...")

        # Internal data
        self.current_pose = None
        self.audio_detection = None
        self.image_data = None
        self.task_instruction = None
        
        # Subscribers
        rospy.Subscriber('/perception/processed_audio', String, self.audio_callback)
        rospy.Subscriber('/slam/estimated_state', PoseStamped, self.estimated_state_callback)
        rospy.Subscriber('/task_executor/task', String, self.task_executor_callback)
        rospy.Subscriber('/perception/image_processing', Image, self.image_processing_callback)

        # Publishers
        self.alert_pub = rospy.Publisher('/victim_detection/alert', String, queue_size=10)
        self.victim_location_pub = rospy.Publisher('/victim_detection/location', Point, queue_size=10)

        self.rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.ready_for_detection():
                self.detect_victims()
            self.rate.sleep()

    def audio_callback(self, msg):
        self.audio_detection = msg.data
        rospy.loginfo_throttle(5.0, "Audio processed data received")

    def estimated_state_callback(self, msg):
        self.current_pose = msg
        rospy.loginfo_throttle(5.0, "Current position received")

    def task_executor_callback(self, msg):
        self.task_instruction = msg.data
        rospy.loginfo("Received task instruction: %s", self.task_instruction)
        
    def image_processing_callback(self, msg):
        self.image_data = msg
        rospy.loginfo_throttle(5.0, "Image processing data received")

    def ready_for_detection(self):
        # We need audio data, image data and current position to detect victims
        return (self.audio_detection is not None and 
                self.current_pose is not None and 
                self.image_data is not None)

    def detect_victims(self):
        rospy.loginfo("Detecting victims based on sensor data...")

        # Simple dummy detection logic
        victim_found = False
        
        # Check if processed audio indicates human voice
        if self.audio_detection and "Human voice detected" in self.audio_detection:
            victim_found = True
            victim_type = "Human voice detected"
        # Check for visual detection based on image processing
        elif self.image_data is not None:
            victim_found = True  # Dummy detection - always find victim in image data
            victim_type = "Visual detection of victim"
        # Random detection (fallback)
        elif rospy.get_time() % 60 < 10:  # Random detection every minute
            victim_found = True
            victim_type = "Combined sensor detection of victim"
        
        if victim_found:
            self.report_victim(victim_type)
        else:
            rospy.loginfo("No victims detected in current scan.")

    def report_victim(self, victim_type):
        # Create alert message with location and detection type
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        location = f"Position: ({x:.2f}, {y:.2f}, {z:.2f})"
        alert_msg = f"ALERT: {victim_type} at {location}"
        
        # Create location point message
        location_point = Point()
        location_point.x = x
        location_point.y = y
        location_point.z = z
        
        # Publish alert and location
        self.alert_pub.publish(alert_msg)
        self.victim_location_pub.publish(location_point)
        
        rospy.logwarn("Published victim alert: %s", alert_msg)
        rospy.logwarn(f"Published victim location: ({x:.2f}, {y:.2f}, {z:.2f})")

if __name__ == '__main__':
    try:
        VictimDetectionAndReportingNode()
    except rospy.ROSInterruptException:
        pass
