#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, Range, LaserScan
from geometry_msgs.msg import WrenchStamped
# Import the FusedData message type (update the package name if needed)
from assignment1.msg import SensorFusion

class StructuralRiskAssessmentNode:
    def __init__(self):
        rospy.init_node('structural_risk_assessment')

        rospy.loginfo("Node initialized. Waiting for assessment request...")

        # Flags to simulate data acquisition
        self.force_data = None
        self.fused_data = None
        # Sensor subscriptions
        rospy.Subscriber('/wrist_right_ft', WrenchStamped, self.force_sensor_callback)
        
        # Subscribe to sensor fusion component
        rospy.Subscriber('/sensor_fusion', SensorFusion, self.sensor_fusion_callback)

        # Rate for the main loop
        self.rate = rospy.Rate(1)  # 1 Hz loop

        while not rospy.is_shutdown():
            if self.ready_for_assessment():
                self.assessment_done = True
                self.assessment_callback()
            self.rate.sleep()

    def ready_for_assessment(self):
        # Check if both force data and fused data are available
        return self.force_data is not None and self.fused_data is not None
        
    def force_sensor_callback(self, msg):
        self.force_data = msg
        rospy.loginfo("Force sensor data received.")

    def sensor_fusion_callback(self, msg):
        rospy.loginfo("Sensor fusion data received.")
        self.fused_data = msg

    def assessment_callback(self):
        rospy.loginfo("Received assessment request.")
        
        processed_data = self.process_data()
        if self.identify_cracks(processed_data):
            risk_level = self.classify_risk()
            if risk_level >= 0.7:
                self.send_alert()
            
            elif risk_level < 0.7:
                self.log_results(risk_level)

            else:
                if self.move_closer():
                    self.classify_risk()
                    if risk_level >= 0.7:
                        self.send_alert()
        else:
            rospy.loginfo("No cracks detected.")

    def process_data(self):
        rospy.loginfo("Processing data coming from sensor fusion node and force sensor...")
        return "processed_data"

    def identify_cracks(self, data):
        rospy.loginfo("Identifying cracks from data...")
        return True  # Always returns True in dummy code

    def classify_risk(self):
        rospy.loginfo("Classifying structural risk...")
        return 0.8  # Dummy risk level

    def send_alert(self):
        rospy.logwarn("High Risk Detected! Sending Alert to Operator.")
        rospy.publish('/supervisor', 'High Risk Detected!')

    def move_closer(self):
        rospy.loginfo("Moving closer to the structure...")
        rospy.publish('/TaskExecuter', 'move_closer')
        return True  # Simulate success

    def log_results(self, risk_level):
        rospy.loginfo(f"[{rospy.get_time()}] Risk Level: {risk_level}")

if __name__ == '__main__':
    try:
        StructuralRiskAssessmentNode()
    except rospy.ROSInterruptException:
        pass
