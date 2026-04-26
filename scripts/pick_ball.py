#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from boeing_gazebo_model_attachment_plugin_msgs.srv import Attach
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
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
            'rg2_finger_joint1', 'rg2_finger_joint2'
        ]
        self.attach_client = self.create_client(Attach, '/attach')
        self.detach_client = self.create_client(Attach, '/detach')
        time.sleep(2)
        self.get_logger().info("Controller initialisiert")

    def move(self, arm_positions, gripper_positions=[0.0, 0.0], wait_time=2.0, duration_sec=2):
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
        self.move(arm_positions, [0.355, 0.355], wait_time=wait_time)

    def attach_cube(self):
        self.get_logger().info("Befestige Würfel am Greifer")
        req = Attach.Request()
        req.joint_name = 'grasp_joint'
        req.model_name_1 = 'ur5_rg2'
        req.link_name_1 = 'rg2_hand'
        req.model_name_2 = 'box'
        req.link_name_2 = 'link'
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if future.result() and future.result().success:
            self.get_logger().info("Würfel befestigt!")
        else:
            self.get_logger().warn("Attach fehlgeschlagen")

    def detach_cube(self):
        self.get_logger().info("Löse Würfel vom Greifer")
        req = Attach.Request()
        req.joint_name = 'grasp_joint'
        req.model_name_1 = 'ur5_rg2'
        req.link_name_1 = 'rg2_hand'
        req.model_name_2 = 'box'
        req.link_name_2 = 'link'
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.get_logger().info("Würfel gelöst!")

def main():
    rclpy.init()
    controller = DirectRobotController()

    # Ausgangsposition
    controller.get_logger().info("Ausgangsposition")
    controller.open_gripper(arm_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_time=2.0)

    # Zwischenposition
    controller.get_logger().info("Zwischenposition")
    controller.open_gripper(arm_positions=[0.0, 0.3, -2.08, -0.15, 1.5, 1.5], wait_time=3.0)

    # Greifposition
    controller.get_logger().info("Greifposition")
    controller.open_gripper(arm_positions=[-0.015, -0.21, -1.95, -0.3, 1.5, -1.622], wait_time=3.0)

    # Greifer schließen
    controller.close_gripper(arm_positions=[-0.015, -0.21, -1.95, -0.3, 1.5, -1.622], wait_time=1.0)

    # Würfel befestigen
    controller.attach_cube()
    time.sleep(0.5)

    # Wurfbewegung 1
    controller.get_logger().info("Wurfbewegung...")
    controller.move([-1.5, 1.3, 0.0, 0.0, 0.0, 0.0], gripper_positions=[0.355, 0.355], wait_time=1.5, duration_sec=1)

    # Wurfbewegung 2 + Würfel loslassen
    controller.get_logger().info("Ball loslassen!")
    controller.detach_cube()
    controller.move([-1.5, -0.6, 0.0, 0.0, 0.0, 0.0], gripper_positions=[1.0, 1.0], wait_time=4.0, duration_sec=1)

    # Zurück zur Ausgangsposition
    controller.get_logger().info("Zurück zur Ausgangsposition")
    controller.open_gripper(arm_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_time=2.0)

    controller.get_logger().info("Fertig!")
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
