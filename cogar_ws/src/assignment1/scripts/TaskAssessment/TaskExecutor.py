#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class TaskExecutorNode:
    def __init__(self):
        rospy.init_node('task_executor')
        rospy.loginfo("Task Executor Node Initialized.")

        # Data storage
        self.current_task = None
        self.mission_plan = None
        self.pending_requests = []
        
        # Subscribers
        rospy.Subscriber('/task_executor/requests', String, self.request_callback)
        rospy.Subscriber('/mission_planner', String, self.mission_planner_callback)
        
        # Publishers
        self.task_pub = rospy.Publisher('/task_executor/task', String, queue_size=10)
        self.status_pub = rospy.Publisher('/task_status', String, queue_size=10)
        
        # Rate for the main loop
        self.rate = rospy.Rate(1)  # 1 Hz loop
        
        # Task execution loop
        while not rospy.is_shutdown():
            self.execute_tasks()
            self.rate.sleep()
            
    def request_callback(self, msg):
        rospy.loginfo(f"Received task request: {msg.data}")
        self.pending_requests.append(msg.data)
        
    def mission_planner_callback(self, msg):
        rospy.loginfo(f"Received mission plan update: {msg.data}")
        self.mission_plan = msg.data
        
    def execute_tasks(self):
        # Process any pending requests first
        if self.pending_requests:
            request = self.pending_requests.pop(0)
            rospy.loginfo(f"Processing request: {request}")
            
            # Handle specific request types
            if request == "move_closer":
                self.current_task = "Moving closer to target"
                self.task_pub.publish("TASK: Moving closer to target")
                self.status_pub.publish("STATUS: Moving")
            elif request == "scan_area":
                self.current_task = "Scanning area for victims"
                self.task_pub.publish("TASK: Scanning area")
                self.status_pub.publish("STATUS: Scanning")
            elif request == "assess_structure":
                self.current_task = "Assessing structural integrity"
                self.task_pub.publish("TASK: Structural assessment")
                self.status_pub.publish("STATUS: Assessing")
            else:
                # Generic task handling
                self.current_task = f"Executing: {request}"
                self.task_pub.publish(f"TASK: {request}")
                self.status_pub.publish(f"STATUS: In progress")
                
        # If no pending requests but we have a mission plan, follow that
        elif self.mission_plan and not self.current_task:
            rospy.loginfo(f"Following mission plan: {self.mission_plan}")
            self.current_task = f"Mission: {self.mission_plan}"
            self.task_pub.publish(f"TASK: Mission {self.mission_plan}")
            self.status_pub.publish("STATUS: Mission in progress")
            
        # Simulate task completion (in real implementation, would have feedback)
        if self.current_task and rospy.get_time() % 10 < 1:  # Complete task every ~10 seconds
            rospy.loginfo(f"Completed task: {self.current_task}")
            self.status_pub.publish("STATUS: Task completed")
            self.current_task = None

if __name__ == '__main__':
    try:
        TaskExecutorNode()
    except rospy.ROSInterruptException:
        pass
