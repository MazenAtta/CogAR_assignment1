#!/usr/bin/env python3
import unittest
import rospy
import rostest
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import WrenchStamped, Twist

class IntegrationTest(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node
        rospy.init_node('integration_test_node')
        
        # Setup publishers
        self.force_pub = rospy.Publisher('/wrist_right_ft', WrenchStamped, queue_size=10)
        self.audio_pub = rospy.Publisher('/microphone/audio', String, queue_size=10)
        self.lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        
        # Setup subscribers
        self.risk_alert = None
        self.task_status = None
        rospy.Subscriber('/risk_alert', String, self.risk_alert_cb)
        rospy.Subscriber('/task_status', String, self.task_status_cb)
        
        # Wait for nodes to initialize
        rospy.sleep(5)

    def risk_alert_cb(self, msg):
        self.risk_alert = msg.data

    def task_status_cb(self, msg):
        self.task_status = msg.data

    def test_full_workflow(self):
        # Test Structural Risk Assessment
        self.force_pub.publish(WrenchStamped())
        rospy.sleep(2)
        
        # Test Victim Detection
        self.audio_pub.publish(String("Help me!"))
        self.lidar_pub.publish(LaserScan())
        rospy.sleep(5)
        
        # Verify Risk Alert
        self.assertIsNotNone(self.risk_alert, "No risk alert published")
        self.assertIn("High Risk", self.risk_alert, "Incorrect risk level")
        
        # Verify Task Execution
        #self.assertIsNotNone(self.task_status, "No task status updates")
        #self.assertIn("STATUS: Moving", self.task_status, "Movement command not executed")

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('assignment1', 'integration_test', IntegrationTest)