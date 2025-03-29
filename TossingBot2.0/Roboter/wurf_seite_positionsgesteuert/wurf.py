#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
import time
import math
import argparse
import threading
from onrobot import RG

class UR5eWaypointController:
    def __init__(self, gripper=None, gripper_ip=None, gripper_port=None):
        rospy.init_node('ur5e_waypoint_controller')
        
        # Define joint names for UR5e
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Connect to the action server
        self.client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for trajectory controller action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to trajectory controller action server")
        
        # Initialize gripper if parameters are provided
        self.gripper = None
        if gripper and gripper_ip and gripper_port:
            try:
                rospy.loginfo(f"Verbindung zum {gripper} Greifer wird hergestellt an {gripper_ip}:{gripper_port}")
                # Konvertieren vom Port zu Integer für Sicherheit
                port_int = int(gripper_port)
                self.gripper = RG(gripper, gripper_ip, port_int)
                
                # Teste die Verbindung durch Abfragen des Status
                status = self.gripper.get_status()
                current_width = self.gripper.get_width_with_offset()
                rospy.loginfo(f"Greifer erfolgreich verbunden. Aktuelle Breite: {current_width} mm")
            except Exception as e:
                rospy.logerr(f"Fehler bei der Verbindung zum Greifer: {e}")
                self.gripper = None
        else:
            rospy.logerr("Greifer-Parameter fehlen oder sind unvollständig. Kein Greifer wird verwendet.")
        
        # Get current joint positions
        self.current_joint_positions = None
        self._get_current_state()
        
        # Position monitoring
        self.position_monitoring_active = False
        self.target_position = None
        self.position_monitor_thread = None
        self.gripper_open_requested = False
    
    def _get_current_state(self):
        """Get the current joint positions from the robot."""
        try:
            joint_states = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
            # Extract the positions for the UR joints
            self.current_joint_positions = [joint_states.position[i] for i in range(6)]
            # Convert to degrees for display
            current_degrees = [math.degrees(pos) for pos in self.current_joint_positions]
            rospy.loginfo(f"Aktuelle Gelenkpositionen (Grad): {[round(deg, 2) for deg in current_degrees]}")
        except Exception as e:
            rospy.logwarn(f"Konnte aktuelle Gelenkpositionen nicht abrufen: {e}")
            # Default values as fallback
            self.current_joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            current_degrees = [math.degrees(pos) for pos in self.current_joint_positions]
            rospy.logwarn(f"Verwende Standardpositionen (Grad): {[round(deg, 2) for deg in current_degrees]}")
    
    def monitor_position(self):
        """Thread function to continuously monitor robot position."""
        rospy.loginfo("Position-Überwachungs-Thread gestartet")
        rate = rospy.Rate(400)  # 200 Hz
        
        # Zähler für Debugging-Ausgaben
        debug_counter = 0
        
        # Zähler für Position nahe am Ziel
        close_to_target_counter = 0
        
        # Zu ignorierende Gelenke (Indizes 0-basiert)
        # 0 = shoulder_pan_joint (erste Koordinate)
        # 2 = elbow_joint (dritte Koordinate)
        ignore_joints = [0, 2]
        
        while self.position_monitoring_active and not rospy.is_shutdown():
            try:
                joint_states = rospy.wait_for_message('/joint_states', JointState, timeout=0.2)
                current_positions = [joint_states.position[i] for i in range(6)]
                current_deg = [math.degrees(pos) for pos in current_positions]
                
                # Debug output every few iterations to avoid flooding logs
                rospy.loginfo(f"Aktuelle Position: {[round(deg, 2) for deg in current_deg]}")
                #if self.target_position:
                    #rospy.loginfo(f"Ziel: {self.target_position}")
                
                # Check if we're near the target position (ignoring joints 0 and 2)
                if self.target_position and not self.gripper_open_requested:
                    # Zähle, wie viele Gelenke (außer den ignorierten) nahe an der Zielposition sind
                    matching_joints = 0
                    total_joints = 0
                    
                    for i in range(6):
                        total_joints += 1
                        # Überprüfen mit größerer Toleranz (5 Grad)
                        if abs(current_deg[i] - self.target_position[i]) < 10.0:
                            matching_joints += 1
                    
                    # Wenn mindestens 75% der nicht-ignorierten Gelenke passen, sind wir nahe am Ziel
                    match_ratio = matching_joints / total_joints if total_joints > 0 else 0
                    
                    if match_ratio >= 0.95:  # 75% oder mehr der betrachteten Gelenke passen
                            rospy.loginfo(f"ZIELPOSITION ERREICHT - ÖFFNE GREIFER ZUM WERFEN! ({matching_joints}/{total_joints} Gelenke passen)")
                            if self.gripper and not self.gripper_open_requested:
                                self.open_gripper()
                                self.gripper_open_requested = True
                                rospy.loginfo("Greifer geöffnet!")
                    else:
                        # Wenn zu weit vom Ziel entfernt, setze den Zähler zurück
                        close_to_target_counter = 0
   
            except Exception as e:
                rospy.logwarn(f"Fehler bei der Positionsüberwachung: {e}")
            
            rate.sleep()
        
        rospy.loginfo("Position-Überwachungs-Thread beendet")
    
    def start_position_monitoring(self, target_position):
        """Start monitoring the robot position for reaching a target."""
        self.target_position = target_position
        self.position_monitoring_active = True
        self.gripper_open_requested = False
        
        # Start the monitoring thread if not already running
        if self.position_monitor_thread is None or not self.position_monitor_thread.is_alive():
            self.position_monitor_thread = threading.Thread(target=self.monitor_position)
            self.position_monitor_thread.daemon = True
            self.position_monitor_thread.start()
            rospy.loginfo(f"Positionsüberwachung gestartet für Zielposition: {target_position}")
    
    def stop_position_monitoring(self):
        """Stop the position monitoring thread."""
        self.position_monitoring_active = False
        if self.position_monitor_thread and self.position_monitor_thread.is_alive():
            self.position_monitor_thread.join(timeout=1.0)
            rospy.loginfo("Positionsüberwachung gestoppt")
    
    def degrees_to_radians(self, degrees_list):
        """Convert a list of angles from degrees to radians."""
        return [math.radians(deg) for deg in degrees_list]
    
    def move_to_joint_positions(self, positions_deg, duration=5.0, velocities=None, accelerations=None):
        """
        Move the robot to the specified joint positions.
        positions_deg: list of 6 joint positions in degrees
        duration: time to take to reach the position in seconds
        velocities: list of 6 joint velocities in radians/sec
        accelerations: list of 6 joint accelerations in radians/sec^2
        """
        # Convert degrees to radians
        positions_rad = self.degrees_to_radians(positions_deg)
        
        rospy.loginfo(f"Bewege zu Positionen (Grad): {positions_deg}")
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions_rad  # Send radians to the robot
        point.time_from_start = rospy.Duration(duration)
        
        # Add velocities if provided
        if velocities is not None:
            rospy.loginfo(f"Verwende angegebene Geschwindigkeiten: {velocities}")
            point.velocities = velocities
            
        # Add accelerations if provided
        if accelerations is not None:
            rospy.loginfo(f"Verwende angegebene Beschleunigungen: {accelerations}")
            point.accelerations = accelerations
        
        trajectory.points.append(point)
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        result = self.client.get_result()
        if result:
            rospy.loginfo("Bewegung erfolgreich abgeschlossen")
            self.current_joint_positions = positions_rad
            return True
        else:
            rospy.logerr("Bewegung fehlgeschlagen")
            return False
    
    def open_gripper(self, force_val=400):
        """Open the gripper if connected."""
        if self.gripper is None:
            rospy.logerr("Kein Greifer verbunden. Kann nicht öffnen.")
            return False
            
        try:
            rospy.loginfo("Öffne Greifer")
            self.gripper.open_gripper(force_val)
            rospy.loginfo("Warte auf Abschluss der Greifer-Bewegung...")
            
            # Warte auf Abschluss der Bewegung mit Timeout
            start_time = time.time()
            timeout = 5.0  # 5 Sekunden Timeout
            
            while time.time() - start_time < timeout:
                try:
                    status = self.gripper.get_status()
                    if not status[0]:  # nicht beschäftigt
                        current_width = self.gripper.get_width_with_offset()
                        rospy.loginfo(f"Greifer geöffnet mit Breite: {current_width} mm")
                        return True
                    time.sleep(0.2)
                except Exception as e:
                    rospy.logerr(f"Fehler beim Abfragen des Greifer-Status: {e}")
                    time.sleep(0.5)
            
            rospy.logwarn("Timeout beim Öffnen des Greifers")
            return False
            
        except Exception as e:
            rospy.logerr(f"Fehler beim Öffnen des Greifers: {e}")
            return False
    
    def close_gripper(self, force_val=400):
        """Close the gripper if connected."""
        if self.gripper is None:
            rospy.logerr("Kein Greifer verbunden. Kann nicht schließen.")
            return False
            
        try:
            rospy.loginfo("Schließe Greifer")
            #self.gripper.close_gripper(force_val)
            self.gripper.move_gripper(400)
            rospy.loginfo("Warte auf Abschluss der Greifer-Bewegung...")
            
            # Warte auf Abschluss der Bewegung mit Timeout
            start_time = time.time()
            timeout = 5.0  # 5 Sekunden Timeout
            
            while time.time() - start_time < timeout:
                try:
                    status = self.gripper.get_status()
                    if not status[0]:  # nicht beschäftigt
                        current_width = self.gripper.get_width_with_offset()
                        rospy.loginfo(f"Greifer geschlossen mit Breite: {current_width} mm")
                        return True
                    time.sleep(0.2)
                except Exception as e:
                    rospy.logerr(f"Fehler beim Abfragen des Greifer-Status: {e}")
                    time.sleep(0.5)
            
            rospy.logwarn("Timeout beim Schließen des Greifers")
            return False
            
        except Exception as e:
            rospy.logerr(f"Fehler beim Schließen des Greifers: {e}")
            return False
    
    def execute_waypoint_sequence(self, waypoints):
        """
        Execute a sequence of waypoints with wait times.
        
        waypoints: list of dictionaries with:
            - 'positions': list of 6 joint positions in degrees
            - 'wait_time': time to wait at this position in seconds
            - 'duration': time to take to reach this position in seconds
            - 'velocities': optional list of joint velocities in radians/sec
            - 'accelerations': optional list of joint accelerations in radians/sec^2
            - 'gripper_action': optional string 'open' or 'close' to control gripper
        """
        # Überprüfen, ob ein Greifer verbunden ist
        if self.gripper is None:
            rospy.logwarn("Kein Greifer verbunden. Führe Bewegungssequenz ohne Greiferaktionen aus.")
        else:
            # Öffne Greifer vor dem ersten Wegpunkt
            rospy.loginfo("Öffne Greifer vor dem Start der Bewegungssequenz")
            self.open_gripper()
        
        # Die spezifische Position, bei der der Greifer öffnen soll
        target_position = [42.96, -50.06, 26.16, 13.88, 100.4, 0.0]
        
        # Starte die Positionsüberwachung mit der Zielposition
        self.start_position_monitoring(target_position)
        
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo(f"Bewege zu Wegpunkt {i+1}/{len(waypoints)}")
            
            # Move to the waypoint
            success = self.move_to_joint_positions(
                waypoint['positions'], 
                duration=waypoint.get('duration', 5.0),
                velocities=waypoint.get('velocities', None),
                accelerations=waypoint.get('accelerations', None)
            )
            
            if not success:
                rospy.logerr(f"Fehler beim Erreichen von Wegpunkt {i+1}")
                self.stop_position_monitoring()
                return False
            
            # Wait at the waypoint
            wait_time = waypoint.get('wait_time', 0.0)
            if wait_time > 0:
                rospy.loginfo(f"Warte an Wegpunkt {i+1} für {wait_time} Sekunden")
                time.sleep(wait_time)
            
            # Wenn dies Wegpunkt 1 ist und ein Greifer verbunden ist, schließe den Greifer
            if i == 0 and self.gripper is not None:
                rospy.loginfo("Schließe Greifer nach Erreichen des ersten Wegpunkts")
                self.close_gripper()
        
        # Stoppe die Positionsüberwachung
        self.stop_position_monitoring()
        
        rospy.loginfo("Wegpunktsequenz abgeschlossen")
        return True
    
    def close_connections(self):
        """Close all connections."""
        # Ensure position monitoring is stopped
        self.stop_position_monitoring()
        
        if self.gripper:
            try:
                self.gripper.close_connection()
                rospy.loginfo("Greifer-Verbindung geschlossen")
            except Exception as e:
                rospy.logerr(f"Fehler beim Schließen der Greifer-Verbindung: {e}")


def get_options():
    """Returns user-specific options."""
    parser = argparse.ArgumentParser(description='Steuerung des UR5e-Roboters und RG2-Greifers.')
    parser.add_argument(
        '--gripper', dest='gripper', type=str,
        default="rg2", choices=['rg2', 'rg6', 'RG2', 'RG6'],
        help='Greifer-Typ, RG2 oder RG6')
    parser.add_argument(
        '--ip', dest='ip', type=str, default="192.168.12.136",
        help='IP-Adresse des Greifers')
    parser.add_argument(
        '--port', dest='port', type=int, default=502,
        help='Port-Nummer des Greifers')
    return parser.parse_args()


def main():
    # Parse command line arguments
    args = get_options()
    
    # Convert gripper name to lowercase for onrobot library
    gripper_type = args.gripper.lower()
    
    rospy.loginfo(f"Starte Programm mit Greifer-Typ: {gripper_type}, IP: {args.ip}, Port: {args.port}")
    
    # Example usage
    try:
        controller = UR5eWaypointController(
            gripper=gripper_type, 
            gripper_ip=args.ip, 
            gripper_port=args.port
        )
        
        # Maximum joint velocity for UR5e in rad/s (approximately 180 deg/s)
        max_velocity = 3.14
        
        # Define your waypoints here with positions in DEGREES
        waypoints = [
            {
                'positions': [0.77, -43.34, 92.68, 36.16, 90.4, 0],  # Point A in degrees
                'wait_time': 1.0,  # Wait 2 seconds at point A
                'duration': 1.5    # Take 1.5 seconds to reach point A
            },
            {
                'positions': [0, -70.62, 83.67, 0, 99.70, 0],  # Point B in degrees
                'wait_time': 1.0,  # Wait 1 second at point B
                'duration': 1.5    # Take 1.5 seconds to reach point B
            },
            {
                'positions': [182.48, -50.67, 103.69, 13.88, 100.40, 0],  # Point C in degrees
                'wait_time': 1.0,  # Wait 1 second at point C
                'duration': 1.5    # Take 1.5 seconds to reach point C
            },
            {
                'positions': [26.16, -50.06, 42.96, 13.88, 100.40, 0],  # Point D in degrees
                'wait_time': 2,  # Wait 2 seconds at point D
                'duration': 0.5,   # Take 0.5 seconds to reach point D for maximum throwing speed
            },
            {
                'positions': [0.77, -43.34, 92.68, 36.16, 90.4, 0],  # Point A in degrees
                'wait_time': 1.0,  # Wait 2 seconds at point A
                'duration': 1.5    # Take 1.5 seconds to reach point A
            }
        ]
        
        controller.execute_waypoint_sequence(waypoints)
        
        # Close all connections at the end
        controller.close_connections()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Programm unterbrochen")
    except Exception as e:
        rospy.logerr(f"Fehler: {e}")


if __name__ == '__main__':
    main() 