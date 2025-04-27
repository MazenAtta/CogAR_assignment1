#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

class ObjectDetectionNode:
    """
    ObjectDetectionNode: A ROS node for detecting objects from processed image data.

    This node subscribes to a topic providing processed image data, detects objects
    based on the received data, and publishes the detected objects' poses.

    Topics:
    =======
        - Subscribed:
            - /perception/image_processing (sensor_msgs.msg.Image): Topic providing processed image data.
        - Published:
            - /perception/detected_objects (geometry_msgs.msg.PoseStamped): Topic for publishing detected objects' poses.

    Attributes:
    ===========
    Attributes:
        - detected_objects_pub (rospy.Publisher): Publisher for detected objects' poses.
        - processed_data (str): Stores the processed image data received from the subscriber.
        - rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
    ========
    Methods:
        - __init__(): Initializes the node, sets up subscribers and publishers, and starts the main loop.
        - processed_image_callback(msg): Callback function for the processed image data subscriber.
        - ready_to_plan(): Checks if the node is ready to detect objects.
        - detect_objects(): Detects objects and publishes their poses.

    """
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
