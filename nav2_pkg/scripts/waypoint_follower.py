#!/usr/bin/env python3
import math
import time
from math import radians
from nav2_simple_commander.robot_navigator import BasicNavigator, PoseStamped, TaskResult
import rclpy
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(yaw):
    """
    Converts a yaw angle (in radians) to a quaternion.
    """
    half_yaw = yaw / 2
    qw = math.cos(half_yaw)
    qz = math.sin(half_yaw)
    return Quaternion(z=qz, w=qw)

def create_goal(x, y, yaw, navigator):
    """
    Creates a PoseStamped object for a given x, y, yaw.
    """
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = navigator.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    quaternion = euler_to_quaternion(yaw)
    goal.pose.orientation = quaternion
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to activate
    navigator.waitUntilNav2Active()

    # Define goal poses
    goal_poses = [
        create_goal(0.0, 0.0, 3.14, navigator),
        create_goal(-2.0, 1.0, 0.97, navigator),
    ]

    # Start navigation
    navigator.followWaypoints(goal_poses)

    # Monitor progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(feedback)

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    else:
        print('Goal failed or has an invalid return status!')

    navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()