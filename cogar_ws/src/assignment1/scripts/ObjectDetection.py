#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node')

        rospy.loginfo("Object Detection Node initialized.")

        # Subscriber to processed image data
        rospy.Subscriber('/image_processing/processed_images', Image, self.processed_image_callback)

        # Publisher for detected objects
        self.detected_objects_pub = rospy.Publisher('/object_detection/detected_objects', PoseStamped, queue_size=10)

        self.processed_data = None

        self.rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.processed_data:
                self.detect_objects()
            self.rate.sleep()

    def processed_image_callback(self, msg):
        rospy.loginfo("Received processed image data.")
        self.processed_data = msg.data

    def detect_objects(self):
        rospy.loginfo("Detecting objects...")
        # Dummy "detected objects" result
        detected_objects = "Detected Object: Dummy Object"
        self.detected_objects_pub.publish(detected_objects)
        rospy.loginfo("Published detected objects.")

if __name__ == '__main__':
    try:
        ObjectDetectionNode()
    except rospy.ROSInterruptException:
        pass
