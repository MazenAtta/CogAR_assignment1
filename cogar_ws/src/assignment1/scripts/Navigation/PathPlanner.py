#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathPlannerNode:
    def __init__(self):
        rospy.init_node('path_planner_node')

        rospy.loginfo("Path Planner Node initialized.")

        # Subscribers
        rospy.Subscriber('/slam/estimated_state', PoseStamped, self.slam_callback)
        rospy.Subscriber('/object_detection', PoseStamped, self.object_detection_callback)
        rospy.Subscriber('/task_executioner', String, self.task_executioner_callback)

        # Publisher
        self.setpoint_pub = rospy.Publisher('/path_planner/setpoint', Path, queue_size=10)

        # Internal data
        self.current_pose = None
        self.task_instruction = None
        self.object_pose = None
        self.target_pose = None

        self.rate = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown():
            if self.ready_to_plan():
                self.plan_path()
            self.rate.sleep()

    def slam_callback(self, msg):
        rospy.loginfo("Received SLAM estimated state.")
        self.current_pose = msg
    
    def object_detection_callback(self, msg):
        rospy.loginfo("Received Object Detection target.")
        self.object_pose = msg
    
    def task_executioner_callback(self, msg):
        rospy.loginfo(f"Received Task Instruction: {msg.data}")
        self.task_instruction = msg.data

    def ready_to_plan(self):
        return self.current_pose is not None and self.object_pose is not None and self.task_instruction is not None

    def plan_path(self):
        rospy.loginfo("Planning path to the target...")
        # Create a dummy path with a single point
        setpoint = Path()
        setpoint.header.stamp = rospy.Time.now()
        setpoint.header.frame_id = "map"
        
        # Create a single waypoint
        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x = 2.0
        waypoint.pose.position.y = 1.5
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0
        
        # Add the waypoint to the path
        setpoint.poses = [waypoint]
        
        self.setpoint_pub.publish(setpoint)
        rospy.loginfo("Published SetPoint to controller.")

if __name__ == '__main__':
    try:
        PathPlannerNode()
    except rospy.ROSInterruptException:
        pass
