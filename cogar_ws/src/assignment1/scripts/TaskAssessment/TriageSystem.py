#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from assignment1.srv import Speaker, SpeakerRequest
from assignment1.msg import SensorFusion

class TriageSystemNode:
    def __init__(self):
        rospy.init_node('triage_system')

        rospy.loginfo("Triage System initialized. Waiting for sensor data...")

        self.rgbd_info = None
        self.vocal_response = None

        # Subscriptions
        rospy.Subscriber('/sensor_fusion', SensorFusion, self.sensor_fusion_callback)
        rospy.Subscriber('/audio_processing/vocal_response', String, self.audio_callback)

        # Publisher
        self.classification_pub = rospy.Publisher('/triage/classification', String, queue_size=10)

        # Wait for speaker service to be available
        rospy.wait_for_service('/speaker')
        self.speaker_client = rospy.ServiceProxy('/speaker', Speaker)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            if self.ready_for_triage():
                self.perform_triage()
            self.rate.sleep()

    def sensor_fusion_callback(self, msg):
        # Extract RGB-D information from the sensor fusion message
        self.rgbd_info = msg.image
        rospy.loginfo(f"RGB-D Info Received")

    def audio_callback(self, msg):
        self.vocal_response = msg.data
        rospy.loginfo(f"Audio Response Received: {self.vocal_response}")
        return True

    def ready_for_triage(self):
        return self.rgbd_info is not None and self.vocal_response is not None

    def perform_triage(self):
        rospy.loginfo("Performing triage assessment...")

        Assessment_question = self.ask_question("Hello, can you hear me?")
        if not Assessment_question:
            rospy.logwarn("Speaker failed to deliver message.")
            return

        rospy.sleep(1)

        conscious = self.analyze_audio_response()
        condition = self.evaluate_condition()

        triage_level = self.classify_victim(conscious, condition)
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

    def analyze_audio_response(self):
        rospy.loginfo("Analyzing vocal response...")
        if self.vocal_response.lower() in ["yes", "i can hear you", "help"]:
            return True
        return False

    def evaluate_condition(self):
        rospy.loginfo("Evaluating physical condition from RGB-D info...")
        if "lying down" in self.rgbd_info.lower():
            return "unconscious"
        return "responsive"

    def classify_victim(self, conscious, condition):
        if not conscious:
            return "Red - Immediate Attention"
        elif condition == "responsive":
            return "Yellow - Delayed"
        else:
            return "Green - Minor"

if __name__ == '__main__':
    try:
        TriageSystemNode()
    except rospy.ROSInterruptException:
        pass
