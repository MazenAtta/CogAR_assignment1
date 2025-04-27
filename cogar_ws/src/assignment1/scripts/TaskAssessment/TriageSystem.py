#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from assignment1.srv import Speaker, SpeakerRequest

class TriageSystemNode:
    def __init__(self):
        rospy.init_node('triage_system')

        rospy.loginfo("Triage System initialized. Waiting for sensor data...")

        # Data holders for the subscribed topics
        self.task_data = None
        self.audio_data = None
        self.image_data = None

        # Subscriptions
        rospy.Subscriber('/task_executor', String, self.task_executor_callback)
        rospy.Subscriber('/perception/processed_audio', String, self.audio_callback)
        rospy.Subscriber('//image_processing', Image, self.image_processing_callback)

        # Wait for speaker service to be available
        rospy.wait_for_service('/speaker')
        self.speaker_client = rospy.ServiceProxy('/speaker', Speaker)

        # Publisher
        self.classification_pub = rospy.Publisher('/triage/classification', String, queue_size=10)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if self.ready_for_triage():
                self.perform_triage()
            self.rate.sleep()

    def task_executor_callback(self, msg):
        self.task_data = msg.data
        rospy.loginfo(f"Task Executor Data Received: {self.task_data}")

    def audio_callback(self, msg):
        self.audio_data = msg.data
        rospy.loginfo(f"Processed Audio Data Received: {self.audio_data}")

    def image_processing_callback(self, msg):
        self.image_data = msg
        rospy.loginfo("Image Processing Data Received")

    def ready_for_triage(self):
        return (self.task_data is not None and 
                self.audio_data is not None and 
                self.image_data is not None)

    def perform_triage(self):
        rospy.loginfo("Performing triage assessment...")

        Assessment_question = self.ask_question("Hello, can you hear me?")
        if not Assessment_question:
            rospy.logwarn("Speaker failed to deliver message.")
            return

        rospy.sleep(1)

        vocal_status = self.analyze_audio_data()
        visual_status = self.analyze_image_data()

        triage_level = self.classify_victim(vocal_status, visual_status)
        rospy.loginfo(f"Triage Result: {triage_level}")
        self.classification_pub.publish(triage_level)

    def ask_question(self, question):
        rospy.loginfo(f"Sending question to speaker: {question}")
        try:
            response = self.speaker_client(SpeakerRequest(message=question))
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def analyze_audio_data(self):
        rospy.loginfo("Analyzing processed audio data...")
        # Dummy analysis - just check if there are any words that indicate consciousness
        if self.audio_data and any(word in self.audio_data.lower() for word in ["yes", "help", "hear"]):
            return "responsive"
        return "non-responsive"

    def analyze_image_data(self):
        rospy.loginfo("Analyzing image processing data...")
        # Dummy analysis - for demonstration purposes only
        # In a real system, would analyze the image data for victim condition
        return "stable"  # Always return stable in this dummy implementation

    def classify_victim(self, vocal_status, visual_status):
        # Dummy triage classification logic
        if vocal_status == "non-responsive":
            classification = "Red - Immediate Attention"
        elif visual_status == "unstable":
            classification = "Yellow - Delayed"
        else:
            classification = "Green - Minor"
            
        # Announce classification using speaker service
        self.ask_question(f"Victim classified as {classification}")
        
        return classification

if __name__ == '__main__':
    try:
        TriageSystemNode()
    except rospy.ROSInterruptException:
        pass
