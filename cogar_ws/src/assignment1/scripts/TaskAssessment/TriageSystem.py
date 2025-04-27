#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from assignment1.srv import Speaker, SpeakerRequest

class TriageSystemNode:
    """
    A ROS node for performing triage assessments based on sensor data.

    This node subscribes to topics providing task, audio, and image data, processes the data,
    and classifies victims into triage levels. It interacts with a speaker service to
    communicate with victims and publishes classification results.

    Topics:
    =======
        - Subscribed:
            - /task_executor/task (String): Task data from the task executor.
            - /perception/processed_audio (String): Processed audio data from the audio processing node.
            - /perception/image_processing (Image): Processed image data from the image processing node.
        - Published:
            - /triage/classification (String): Triage classification results.
        - Service:
            - /speaker (Speaker): Service for interacting with the speaker.

    Attributes:
    ===========
    Attributes:
        task_data (str): Task data received from the subscriber.
        audio_data (str): Processed audio data received from the subscriber.
        image_data (sensor_msgs.msg.Image): Processed image data received from the subscriber.
        speaker_client (rospy.ServiceProxy): Client for interacting with the speaker service.
        classification_pub (rospy.Publisher): Publishes triage classification results.
        rate (rospy.Rate): Controls the loop rate of the node.

    Methods:
    ========
    Methods:
        __init__():
            Initializes the node, sets up subscribers, publishers, service clients, and starts the main loop.
        task_executor_callback(msg):
            Callback function for receiving task data.
        audio_callback(msg):
            Callback function for receiving processed audio data.
        image_processing_callback(msg):
            Callback function for receiving processed image data.
        ready_for_triage():
            Checks if the node has received all necessary data for triage.
        perform_triage():
            Performs the triage assessment and publishes the classification result.
        ask_question(question):
            Sends a question to the speaker service and returns success status.
        analyze_audio_data():
            Analyzes the processed audio data to determine vocal responsiveness.
        analyze_image_data():
            Analyzes the processed image data to determine visual status.
        classify_victim(vocal_status, visual_status):
            Classifies the victim based on vocal and visual status.
    """

    def __init__(self):
        rospy.init_node('triage_system')

        rospy.loginfo("Triage System initialized. Waiting for sensor data...")

        # Data holders for the subscribed topics
        self.task_data = None
        self.audio_data = None
        self.image_data = None

        # Subscriptions
        rospy.Subscriber('/task_executor/task', String, self.task_executor_callback)
        rospy.Subscriber('/perception/processed_audio', String, self.audio_callback)
        rospy.Subscriber('/perception/image_processing', Image, self.image_processing_callback)

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
            

        vocal_status = self.analyze_audio_data()
        visual_status = self.analyze_image_data()

        triage_level = self.classify_victim(vocal_status, visual_status)
        rospy.logwarn(f"Triage Result: {triage_level}")
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
