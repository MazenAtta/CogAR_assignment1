#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, String

class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node')

        rospy.loginfo("Image Processing Node initialized.")

        # Subscribers to RGB and Depth camera
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)

        # Publisher for processed image data
        self.processed_image_pub = rospy.Publisher('/image_processing', Image, queue_size=10)

        self.rgb_image = None
        self.depth_image = None

        self.rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown():
            if self.rgb_image and self.depth_image:
                self.process_images()
            self.rate.sleep()

    def rgb_callback(self, msg):
        rospy.loginfo("Received RGB image.")
        self.rgb_image = msg

    def depth_callback(self, msg):
        rospy.loginfo("Received Depth image.")
        self.depth_image = msg

    def process_images(self):
        rospy.loginfo("Processing RGB and Depth images...")
        # Dummy "processed" output
        processed_result = "Processed Image Data"
        self.processed_image_pub.publish(processed_result)
        rospy.loginfo("Published processed image data.")

if __name__ == '__main__':
    try:
        ImageProcessingNode()
    except rospy.ROSInterruptException:
        pass
