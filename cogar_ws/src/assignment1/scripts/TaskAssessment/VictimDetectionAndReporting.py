#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from assignment1.msg import SensorFusion

class VictimDetectionAndReportingNode:
    def __init__(self):
        rospy.init_node('victim_detection_and_reporting')

        rospy.loginfo("Victim Detection Node Initialized. Waiting for sensor data...")

        self.rgbd_info = None

        # Subscribe to sensor fusion data
        rospy.Subscriber('/sensor_fusion', SensorFusion, self.sensor_fusion_callback)

        # Publisher to inform TriageSystem (could also notify supervisor or task manager)
        self.victim_pub = rospy.Publisher('/victim_detection/found_victims', String, queue_size=10)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if self.ready_for_detection():
                self.detect_victims()
            self.rate.sleep()

    def sensor_fusion_callback(self, msg):
        # Assume RGB-D camera info is embedded in SensorFusion message
        self.rgbd_info = msg.image
        rospy.loginfo(f"RGB-D Info received for victim detection")

    def ready_for_detection(self):
        return self.rgbd_info is not None

    def detect_victims(self):
        rospy.loginfo("Detecting victims based on sensor data...")

        victim_found = self.process_rgbd_data()

        if victim_found:
            self.report_victim()
        else:
            rospy.loginfo("No victims detected in current scan.")

    def process_rgbd_data(self):
        rospy.loginfo("Processing RGB-D data for victim detection...")
        # Simulated victim detection logic
        # In a real scenario, this would involve image processing and possibly machine learning
        return True

    def report_victim(self):
        rospy.logwarn("Victim detected! Reporting to system...")
        self.victim_pub.publish("Victim Detected at Location XYZ")

if __name__ == '__main__':
    try:
        VictimDetectionAndReportingNode()
    except rospy.ROSInterruptException:
        pass
