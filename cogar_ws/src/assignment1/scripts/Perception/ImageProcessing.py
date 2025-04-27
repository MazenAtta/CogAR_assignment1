#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

class ImageProcessingNode:
    """
    ImageProcessingNode: A ROS node for processing RGB and Depth images.

    This node subscribes to RGB-D camera, processes the images, and publishes the processed image data to a specified topic.

    Topics:
        - Subscribed:
            - /xtion/rgb/image_raw (sensor_msgs.msg.Image): RGB image data from the camera.
            - /xtion/depth/image_raw (sensor_msgs.msg.Image): Depth image data from the camera.
        - Published:
            - /perception/image_processing (sensor_msgs.msg.Image): Processed image data.

    Attributes:
        rgb_image (sensor_msgs.msg.Image): Stores the latest RGB image received from the camera.
        depth_image (sensor_msgs.msg.Image): Stores the latest Depth image received from the camera.
        processed_image_pub (rospy.Publisher): Publishes the processed image data to the `/perception/image_processing` topic.

    Methods:
        __init__():
            Initializes the ROS node, sets up subscribers and publishers, and starts the processing loop.

        rgb_callback(msg):
            Callback function for the RGB image topic. Stores the received RGB image.

            Args:
                msg (sensor_msgs.msg.Image): The RGB image message received from the topic.

        depth_callback(msg):
            Callback function for the Depth image topic. Stores the received Depth image.

            Args:
                msg (sensor_msgs.msg.Image): The Depth image message received from the topic.

        process_images():
            Processes the RGB and Depth images and publishes the processed result.

            The processed result is a dummy output that copies the RGB image data and publishes it
            with updated metadata.

            Returns:
                None
    """
    def __init__(self):
        rospy.init_node('image_processing_node')

        rospy.loginfo("Image Processing Node initialized.")

        # Subscribers to RGB and Depth camera
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)

        # Publisher for processed image data
        self.processed_image_pub = rospy.Publisher('/perception/image_processing', Image, queue_size=10)

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
        processed_result = Image()
        processed_result.header.stamp = rospy.Time.now()
        processed_result.header.frame_id = "processed_image_frame"
        processed_result.height = self.rgb_image.height
        processed_result.width = self.rgb_image.width
        processed_result.encoding = self.rgb_image.encoding
        processed_result.is_bigendian = self.rgb_image.is_bigendian
        processed_result.step = self.rgb_image.step
        processed_result.data = self.rgb_image.data  # Dummy data, copying RGB image data
        self.processed_image_pub.publish(processed_result)
        rospy.loginfo("Published processed image data.")

if __name__ == '__main__':
    try:
        ImageProcessingNode()
    except rospy.ROSInterruptException:
        pass
