#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, Range, LaserScan
from geometry_msgs.msg import WrenchStamped, PoseStamped
from assignment1.msg import SensorFusion

class StructuralRiskAssessmentNode:
    def __init__(self):
        rospy.init_node('structural_risk_assessment')

        rospy.loginfo("Node initialized. Waiting for assessment request...")

        # Flags to simulate data acquisition
        self.force_data = None
        self.sensor_fusion_data = None
        self.task_executor_data = None
        self.image_processing_data = None
        
        # Sensor subscriptions
        rospy.Subscriber('/wrist_right_ft', WrenchStamped, self.force_sensor_callback)
        rospy.Subscriber('/perception/sensor_fusion', SensorFusion, self.sensor_fusion_callback)
        rospy.Subscriber('/task_executor', String, self.task_executor_callback)
        rospy.Subscriber('/perception/image_processing', Image, self.image_processing_callback)
        
        # Publishers
        self.alert_publisher = rospy.Publisher('/risk_alert', String, queue_size=10)
        self.risk_level_publisher = rospy.Publisher('/risk_level', Float32, queue_size=10)
        
        # Service client for requesting movement
        self.move_closer_service = rospy.ServiceProxy('/task_executor/move_closer', Speaker)
        
        # Rate for the main loop
        self.rate = rospy.Rate(1)  # 1 Hz loop

        while not rospy.is_shutdown():
            if self.ready_for_assessment():
                self.assessment_callback()
            self.rate.sleep()

    def ready_for_assessment(self):
        # Check if all required data is available
        return (self.force_data is not None and 
                self.sensor_fusion_data is not None and 
                self.task_executor_data is not None and 
                self.image_processing_data is not None)
        
    def force_sensor_callback(self, msg):
        self.force_data = msg
        rospy.loginfo("Force sensor data received.")
        
    def sensor_fusion_callback(self, msg):
        self.sensor_fusion_data = msg
        rospy.loginfo("Sensor fusion data received.")
        
    def task_executor_callback(self, msg):
        self.task_executor_data = msg
        rospy.loginfo("Task executor data received.")
        
    def image_processing_callback(self, msg):
        self.image_processing_data = msg
        rospy.loginfo("Image processing data received.")

    def assessment_callback(self):
        rospy.loginfo("Performing structural risk assessment.")
        
        processed_data = self.process_data()
        if self.identify_cracks(processed_data):
            risk_level = self.classify_risk()
            if risk_level >= 0.7:
                self.send_alert(risk_level)
            
            elif risk_level < 0.7:
                self.log_results(risk_level)
                self.risk_level_publisher.publish(Float32(risk_level))

            else:
                if self.request_move_closer():
                    risk_level = self.classify_risk()
                    if risk_level >= 0.7:
                        self.send_alert(risk_level)
        else:
            rospy.loginfo("No cracks detected.")

    def process_data(self):
        rospy.loginfo("Processing data from sensor fusion, force sensor, task executor, and image processing...")
        # This is a dummy function - no actual processing is done
        return "processed_data"

    def identify_cracks(self, data):
        rospy.loginfo("Identifying potential structural issues...")
        # Dummy function that always detects issues
        return True

    def classify_risk(self):
        rospy.loginfo("Classifying structural risk level...")
        # Dummy risk classification that returns a high risk value
        return 0.8

    def send_alert(self, risk_level):
        rospy.logwarn(f"ALERT: High Risk Level {risk_level} Detected!")
        self.alert_publisher.publish(f"ALERT: Risk Level {risk_level}")
        self.risk_level_publisher.publish(Float32(risk_level))

    def request_move_closer(self):
        rospy.loginfo("Requesting robot to move closer to the structure...")
        try:
            response = self.move_closer_service("move_closer")
            rospy.loginfo(f"Move closer request response: {response.success}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def log_results(self, risk_level):
        rospy.loginfo(f"[{rospy.get_time()}] Current Risk Level: {risk_level}")

if __name__ == '__main__':
    try:
        StructuralRiskAssessmentNode()
    except rospy.ROSInterruptException:
        pass