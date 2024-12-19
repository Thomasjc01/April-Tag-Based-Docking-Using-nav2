#!/usr/bin/env python3
import math
import time
from math import radians
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import do_transform_pose

class DockingNavigator(Node):
    def __init__(self):
        super().__init__('docking_navigator')
        
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Create navigator
        self.navigator = BasicNavigator()
        
        # Set up timer to check for AprilTag
        self.create_timer(1.0, self.check_tag_and_navigate)
        
        # Flag to track if navigation has started
        self.navigation_started = False

    def euler_to_quaternion(self, yaw):
        half_yaw = yaw / 2
        return Quaternion(z=math.sin(half_yaw), w=math.cos(half_yaw))

    def create_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = self.euler_to_quaternion(yaw)
        return goal

    def check_tag_and_navigate(self):
        if self.navigation_started:
            return

        try:
            # Get transform from camera_link to AprilTag
            transform = self.tf_buffer.lookup_transform(
                'map',
                'tag36h11:0',
                rclpy.time.Time())
            
            # Extract position from transform
            tag_x = transform.transform.translation.x
            tag_y = transform.transform.translation.y
            
            # Create waypoint slightly in front of tag
            offset_distance = 0.5  # meters
            goal_pose = self.create_goal(tag_x - offset_distance, tag_y, 0.0)
            
            # Start navigation
            self.navigator.goToPose(goal_pose)
            self.navigation_started = True
            
            # Monitor progress
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                self.get_logger().info(f'Navigation feedback: {feedback}')
            
            # Check result
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
            else:
                self.get_logger().error('Navigation failed!')
                
        except TransformException as ex:
            self.get_logger().debug(f'Could not get transform: {ex}')
            return

def main():
    rclpy.init()
    
    navigator_node = DockingNavigator()
    
    # Wait for Nav2 to be ready
    navigator_node.navigator.waitUntilNav2Active()
    
    try:
        rclpy.spin(navigator_node)
    except KeyboardInterrupt:
        pass
    
    navigator_node.navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
