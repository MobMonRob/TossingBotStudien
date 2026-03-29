#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math
import time
import threading

class UR5eBaseController(Node):
    def __init__(self, node_name='ur5e_controller', gripper_ip=None, gripper_port=502):
        super().__init__(node_name)

        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.get_logger().info("Warte auf Action Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Action Server verbunden")

        # Greifer via onrobot (direkt per IP, kein ROS)
        self.gripper = None
        if gripper_ip:
            try:
                from onrobot import RG
                self.gripper = RG('rg2', gripper_ip, gripper_port)
                self.get_logger().info(f"Greifer verbunden: {gripper_ip}:{gripper_port}")
            except Exception as e:
                self.get_logger().error(f"Greifer-Fehler: {e}")

        self.current_joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._get_current_state()

        # Positionsüberwachung
        self.position_monitoring_active = False
        self.target_position = None
        self.gripper_open_requested = False
        self.position_monitor_thread = None

    def _get_current_state(self):
        try:
            msg = self.wait_for_message('/joint_states', JointState, timeout=5.0)
            self.current_joint_positions = list(msg.position[:6])
            degs = [round(math.degrees(p), 2) for p in self.current_joint_positions]
            self.get_logger().info(f"Aktuelle Position (Grad): {degs}")
        except Exception as e:
            self.get_logger().warn(f"Gelenkpositionen nicht abrufbar: {e} — Verwende Standardwerte")

    def wait_for_message(self, topic, msg_type, timeout=5.0):
        """Wartet auf eine einzige Nachricht auf einem Topic."""
        result = [None]
        event = threading.Event()
        sub = self.create_subscription(msg_type, topic,
            lambda msg: (result.__setitem__(0, msg), event.set()), 10)
        event.wait(timeout=timeout)
        self.destroy_subscription(sub)
        if result[0] is None:
            raise TimeoutError(f"Kein Message auf {topic} binnen {timeout}s")
        return result[0]

    def degrees_to_radians(self, deg_list):
        return [math.radians(d) for d in deg_list]

    def move_to_joint_positions(self, positions_deg, duration=5.0, velocities=None, accelerations=None):
        positions_rad = self.degrees_to_radians(positions_deg)
        self.get_logger().info(f"Bewege zu (Grad): {positions_deg}")

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions_rad
        point.time_from_start = Duration(sec=int(duration),
                                         nanosec=int((duration % 1) * 1e9))
        if velocities:
            point.velocities = velocities
        if accelerations:
            point.accelerations = accelerations
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal abgelehnt")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.current_joint_positions = positions_rad
        self.get_logger().info("Bewegung abgeschlossen")
        return True

    def open_gripper(self, force_val=400):
        if not self.gripper:
            self.get_logger().warn("Kein Greifer verbunden")
            return False
        try:
            self.gripper.open_gripper(force_val)
            self._wait_for_gripper()
            return True
        except Exception as e:
            self.get_logger().error(f"Greifer öffnen fehlgeschlagen: {e}")
            return False

    def close_gripper(self, force_val=400):
        if not self.gripper:
            self.get_logger().warn("Kein Greifer verbunden")
            return False
        try:
            self.gripper.move_gripper(force_val)
            self._wait_for_gripper()
            return True
        except Exception as e:
            self.get_logger().error(f"Greifer schließen fehlgeschlagen: {e}")
            return False

    def _wait_for_gripper(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            try:
                if not self.gripper.get_status()[0]:
                    return True
            except:
                pass
            time.sleep(0.2)
        self.get_logger().warn("Greifer Timeout")
        return False

    def monitor_position(self, target_deg):
        """Thread: Überwacht Position und öffnet Greifer bei Ziel."""
        self.get_logger().info(f"Positionsüberwachung gestartet für: {target_deg}")
        rate_sec = 1.0 / 400.0
        while self.position_monitoring_active:
            try:
                msg = self.wait_for_message('/joint_states', JointState, timeout=0.2)
                current_deg = [math.degrees(msg.position[i]) for i in range(6)]
                if not self.gripper_open_requested:
                    match = sum(1 for i in range(6)
                                if abs(current_deg[i] - target_deg[i]) < 10.0)
                    if match / 6 >= 0.95:
                        self.get_logger().info("Zielposition erreicht — öffne Greifer!")
                        self.open_gripper()
                        self.gripper_open_requested = True
            except:
                pass
            time.sleep(rate_sec)

    def start_position_monitoring(self, target_deg):
        self.target_position = target_deg
        self.position_monitoring_active = True
        self.gripper_open_requested = False
        self.position_monitor_thread = threading.Thread(
            target=self.monitor_position, args=(target_deg,), daemon=True)
        self.position_monitor_thread.start()

    def stop_position_monitoring(self):
        self.position_monitoring_active = False
        if self.position_monitor_thread:
            self.position_monitor_thread.join(timeout=1.0)

    def execute_waypoint_sequence(self, waypoints, release_target_deg=None):
        if self.gripper:
            self.open_gripper()

        if release_target_deg:
            self.start_position_monitoring(release_target_deg)

        for i, wp in enumerate(waypoints):
            self.get_logger().info(f"Wegpunkt {i+1}/{len(waypoints)}")
            ok = self.move_to_joint_positions(
                wp['positions'],
                duration=wp.get('duration', 5.0),
                velocities=wp.get('velocities'),
                accelerations=wp.get('accelerations')
            )
            if not ok:
                self.get_logger().error(f"Wegpunkt {i+1} fehlgeschlagen")
                self.stop_position_monitoring()
                return False
            if wp.get('wait_time', 0) > 0:
                time.sleep(wp['wait_time'])
            if i == 0 and self.gripper:
                self.close_gripper()

        self.stop_position_monitoring()
        self.get_logger().info("Sequenz abgeschlossen")
        return True

    def close_connections(self):
        self.stop_position_monitoring()
        if self.gripper:
            try:
                self.gripper.close_connection()
            except:
                pass
