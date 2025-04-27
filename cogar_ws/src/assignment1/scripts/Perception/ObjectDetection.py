#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node')

        rospy.loginfo("Object Detection Node initialized.")

        # Subscriber to processed image data
        rospy.Subscriber('/perception/image_processing', Image, self.processed_image_callback)

        # Publisher for detected objects
        self.detected_objects_pub = rospy.Publisher('/perception/detected_objects', PoseStamped, queue_size=10)

        self.processed_data = None

        self.rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.processed_data:
                self.detect_objects()
            self.rate.sleep()

    def processed_image_callback(self, msg):
        rospy.loginfo("Received processed image data.")
        self.processed_data = msg.data

    def ready_to_plan(self):
        return self.processed_data is not None

    def detect_objects(self):
        while not self.ready_to_plan():
            rospy.loginfo("Waiting for processed image data...")
            self.rate.sleep()

        rospy.loginfo("Detecting objects...")
        # Dummy "detected objects" result
        detected_objects = PoseStamped()
        detected_objects.header.stamp = rospy.Time.now()
        detected_objects.header.frame_id = "camera_frame"
        detected_objects.pose.position.x = 1.0
        detected_objects.pose.position.y = 2.0
        detected_objects.pose.position.z = 3.0
        detected_objects.pose.orientation.x = 0.0
        detected_objects.pose.orientation.y = 0.0
        detected_objects.pose.orientation.z = 0.0
        detected_objects.pose.orientation.w = 1.0
        self.detected_objects_pub.publish(detected_objects)
        rospy.loginfo("Published detected objects.")

if __name__ == '__main__':
    try:
        ObjectDetectionNode()
    except rospy.ROSInterruptException:
        pass
