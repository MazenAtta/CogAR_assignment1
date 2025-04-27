#!/usr/bin/env python3
import unittest
import rospy
import rostest
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from geometry_msgs.msg import WrenchStamped, Twist, PoseStamped, Point
from nav_msgs.msg import Path
from assignment1.msg import SensorFusion

class IntegrationTest(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node
        rospy.init_node('integration_test_node')
        
        # Setup publishers
        self.audio_pub = rospy.Publisher('/microphone/audio', String, queue_size=10)
        
        # Setup subscribers
        self.risk_alert = None
        self.left_motor_status = None
        self.right_motor_status = None
        self.task_status = None
        self.fused_data = None
        self.pointcloud_data = None
        self.detected_objects = None
        self.processed_image = None
        self.estimated_state = None
        self.setpoint = None

        rospy.Subscriber('/risk_alert', String, self.risk_alert_cb)
        rospy.Subscriber('/victim_detection/alert', String, self.victim_alert_cb)
        rospy.Subscriber('/triage/classification', String, self.classification_cb)
        rospy.Subscriber('/task_status', String, self.task_status_cb)
        rospy.Subscriber('/motor_driver/left/status', String, self.left_motor_status_cb)
        rospy.Subscriber('/motor_driver/right/status', String, self.right_motor_status_cb)
        rospy.Subscriber('/perception/sensor_fusion', SensorFusion, self.fused_data_cb)
        rospy.Subscriber('/perception/pointcloud_processing', PointCloud2, self.pointcloud_cb)
        rospy.Subscriber('/perception/detected_objects', PoseStamped, self.detected_objects_cb)
        rospy.Subscriber('/perception/image_processing', Image, self.processed_image_cb)
        rospy.Subscriber('/slam/estimated_state', PoseStamped, self.estimated_state_cb)
        rospy.Subscriber('/path_planner/setpoint', Path, self.setpoint_cb)
        
        # Wait for nodes to initialize
        rospy.sleep(5)

    def risk_alert_cb(self, msg):
        self.risk_alert = msg.data

    def classification_cb(self, msg):
        self.classification = msg.data

    def task_status_cb(self, msg):
        self.task_status = msg.data

    def left_motor_status_cb(self, msg):
        self.left_motor_status = msg.data

    def right_motor_status_cb(self, msg):
        self.right_motor_status = msg.data

    def fused_data_cb(self, msg):
        self.fused_data = msg

    def pointcloud_cb(self, msg):
        self.pointcloud_data = msg

    def detected_objects_cb(self, msg):
        self.detected_objects = msg

    def processed_image_cb(self, msg):
        self.processed_image = msg

    def estimated_state_cb(self, msg):
        self.estimated_state = msg

    def setpoint_cb(self, msg):
        self.setpoint = msg
    
    def victim_alert_cb(self, msg):
        self.victim_alert = msg.data

    def test_full_workflow(self):

        
        # Test Victim Detection
        self.audio_pub.publish(String("Help me!"))
        rospy.sleep(10)
        
        # Test Risk Assessment
        self.assertIsNotNone(self.risk_alert, "No risk alert published")
        self.assertIn("High Risk", self.risk_alert, "Incorrect risk level")

        # Test Triage Classification
        self.assertIsNotNone(self.classification, "No classification published")
        self.assertIn("Red - Immediate Attention", self.classification, "Incorrect classification")

        # Test Victim Detection
        self.assertIsNotNone(self.victim_alert, "No detected objects published")
        self.assertIn("ALERT: Human voice detected at Position: (0.00, 0.00, 0.00)", self.victim_alert, "No victim detected")

        # Test Task Executor
        #self.assertIsNotNone(self.task_status, "No task status published")
        #self.assertIn("TASK: None", self.task_status, "Task not completed")

    def tearDown(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('assignment1', 'integration_test', IntegrationTest)