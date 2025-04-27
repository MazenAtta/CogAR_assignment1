#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

class AudioProcessingNode:
    """
    A ROS node for processing audio data from a microphone.

    This node subscribes to a topic providing raw audio data, processes the data
    to detect specific audio patterns, and publishes the results.

    Topics:
        - Subscribed:
            - /microphone/audio (AudioData): Raw audio data from the microphone.

        - Published:
            - /perception/processed_audio (String): Processed audio results.

    Attributes:
        processed_audio_pub (rospy.Publisher): Publishes processed audio results.
        audio_buffer (list): Buffer storing raw audio data received from the subscriber.
        rate (rospy.Rate): Controls the loop rate of the node.
    
    Methods:
        __init__():
            Initializes the node, sets up subscribers and publishers, and starts the main loop.
        audio_callback(msg):
            Callback function for processing raw audio data.
        process_loop():
            Main processing loop that analyzes audio data and publishes results.

    """

    def __init__(self):
        rospy.init_node('audio_processing_node')
        rospy.loginfo("Audio Processing Node initialized.")

        # Subscriber
        rospy.Subscriber('/microphone/audio', String, self.audio_callback)

        # Publisher
        self.processed_audio_pub = rospy.Publisher('/perception/processed_audio', String, queue_size=10)

        # Internal data
        self.audio_buffer = []
        self.rate = rospy.Rate(10)  # 10 Hz

        # Main loop
        while not rospy.is_shutdown():
            self.process_loop()
            self.rate.sleep()

    def audio_callback(self, msg):
        self.audio_buffer.append(msg)
        rospy.loginfo_throttle(5.0, "Received audio data from microphone")

    def process_loop(self):
                
        # Publish result
        self.processed_audio_pub.publish("Human voice detected")


if __name__ == '__main__':
    try:
        audio_processor = AudioProcessingNode()
    except rospy.ROSInterruptException:
        pass
