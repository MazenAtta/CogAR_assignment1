#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class VictimDetectionAndReportingNode:
    def __init__(self):
        rospy.init_node('victim_detection_and_reporting')
        rospy.loginfo("Victim Detection Node Initialized. Waiting for sensor data...")

        # Internal data
        self.current_pose = None
        self.audio_detection = None
        self.task_instruction = None
        
        # Subscribers
        rospy.Subscriber('/perception/processed_audio', String, self.audio_callback)
        rospy.Subscriber('/slam/estimated_state', PoseStamped, self.estimated_state_callback)
        rospy.Subscriber('/task_executor', String, self.task_executor_callback)

        # Publisher
        self.alert_pub = rospy.Publisher('/victim_detection/alert', String, queue_size=10)

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

    def ready_for_detection(self):
        # We need audio data and current position to detect victims
        return self.audio_detection is not None and self.current_pose is not None

    def detect_victims(self):
        rospy.loginfo("Detecting victims based on sensor data...")

        # Simple dummy detection logic
        victim_found = False
        
        # Check if processed audio indicates human voice
        if self.audio_detection and "Human voice" in self.audio_detection:
            victim_found = True
            victim_type = "Human voice detected"
        elif rospy.get_time() % 60 < 10:  # Random detection every minute
            victim_found = True
            victim_type = "Visual detection of victim"
        
        if victim_found:
            self.report_victim(victim_type)
        else:
            rospy.loginfo("No victims detected in current scan.")

    def report_victim(self, victim_type):
        # Create alert message with location and detection type
        location = f"Position: ({self.current_pose.pose.position.x:.2f}, {self.current_pose.pose.position.y:.2f})"
        alert_msg = f"ALERT: {victim_type} at {location}"
        
        # Publish alert
        self.alert_pub.publish(alert_msg)
        rospy.loginfo("Published victim alert: %s", alert_msg)

if __name__ == '__main__':
    try:
        VictimDetectionAndReportingNode()
    except rospy.ROSInterruptException:
        pass
