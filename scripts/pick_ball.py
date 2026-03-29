#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class DirectRobotController(Node):
    def __init__(self):
        super().__init__('direct_robot_controller')
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.all_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
            'rg2_finger_joint1',
            'rg2_finger_joint2'
        ]
        
        time.sleep(2)
        self.get_logger().info("Controller initialisiert")
    
    def move(self, arm_positions, gripper_positions=[0.0, 0.0], wait_time=2.0, duration_sec=2):
        """Bewege Arm und Greifer zusammen"""
        msg = JointTrajectory()
        msg.joint_names = self.all_joints
        
        point = JointTrajectoryPoint()
        point.positions = arm_positions + gripper_positions
        point.time_from_start = Duration(sec=duration_sec)
        msg.points = [point]
        
        self.get_logger().info(f"Arm: {arm_positions} | Greifer: {gripper_positions}")
        self.publisher.publish(msg)
        time.sleep(wait_time)
    
    def open_gripper(self, arm_positions=[0.0,0.0,0.0,0.0,0.0,0.0], wait_time=1.0):
        self.get_logger().info("Öffne Greifer")
        self.move(arm_positions, [1.0, 1.0], wait_time=wait_time)
    
    def close_gripper(self, arm_positions=[0.0,0.0,0.0,0.0,0.0,0.0], wait_time=1.0):
        self.get_logger().info("Schließe Greifer")
        self.move(arm_positions, [0.5, 0.5], wait_time=wait_time)


def main():
    rclpy.init()
    controller = DirectRobotController()
    
    # Ausgangsposition + Greifer öffnen
    controller.get_logger().info("Ausgangsposition")
    controller.open_gripper(arm_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_time=2.0)
    
    # Greifposition
    controller.get_logger().info("Greifposition")
    controller.move([0.0, -0.15, -2.08, -0.15, 1.5, 1.5], wait_time=2.0)
    
    # Greifer schließen
    controller.close_gripper(arm_positions=[0.0, -0.15, -2.08, -0.15, 1.5, 1.5], wait_time=1.0)
    
    # Wurfbewegung 1
    controller.get_logger().info("Wurfbewegung...")
    controller.move([-1.5, 1.3, 0.0, 0.0, 0.0, 0.0], gripper_positions=[0.5, 0.5], wait_time=1.5, duration_sec=1)
    
    # Wurfbewegung 2 - schnell + Greifer öffnen!
    controller.get_logger().info("Ball loslassen!")
    controller.move([-1.5, -0.6, 0.0, 0.0, 0.0, 0.0], gripper_positions=[1.0, 1.0], wait_time=4.0, duration_sec=1)
    
    # Zurück zur Ausgangsposition
    controller.get_logger().info("Zurück zur Ausgangsposition")
    controller.open_gripper(arm_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_time=2.0)
    
    controller.get_logger().info("Fertig!")
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
