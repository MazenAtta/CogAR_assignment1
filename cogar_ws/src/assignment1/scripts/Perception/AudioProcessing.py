#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

class AudioProcessingNode:
    def __init__(self):
        rospy.init_node('audio_processing_node')
        rospy.loginfo("Audio Processing Node initialized.")

        # Subscriber
        rospy.Subscriber('/microphone/audio', AudioData, self.audio_callback)

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
        """Receive raw audio data from microphone"""
        self.audio_buffer.append(msg)
        rospy.loginfo_throttle(5.0, "Received audio data from microphone")

    def process_loop(self):
        """Simple processing loop for audio data"""
        if not self.audio_buffer:
            return  # No data to process
        
        # Simple dummy detection - pretend we found something
        if len(self.audio_buffer) > 5:
            # Just a dummy result
            if rospy.get_time() % 30 < 10:  # Randomly detect something every 30 seconds
                result = "Human voice detected"
            elif rospy.get_time() % 30 < 20:
                result = "Structural noise detected"
            else:
                result = "No significant audio detected"
                
            # Publish result
            self.processed_audio_pub.publish(result)
            rospy.loginfo("Audio processing result: %s", result)
            
            # Clear buffer after processing
            self.audio_buffer = []

if __name__ == '__main__':
    try:
        audio_processor = AudioProcessingNode()
    except rospy.ROSInterruptException:
        pass
