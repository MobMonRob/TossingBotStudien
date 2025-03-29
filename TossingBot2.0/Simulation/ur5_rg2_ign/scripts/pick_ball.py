#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from math import pi
import time

class DirectRobotController:
    def __init__(self):
        # Initialisiere den Node
        rospy.init_node('direct_robot_controller', anonymous=True)
        
        # Definiere die Joint-Namen
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Erstelle Publisher für die Arm-Gelenke
        self.joint_publishers = {}
        for joint_name in self.joint_names:
            self.joint_publishers[joint_name] = rospy.Publisher(
                f'/{joint_name}_position_controller/command', 
                Float64, 
                queue_size=10
            )
        
        # Erstelle Publisher für die Greifer-Gelenke
        self.pub_finger1 = rospy.Publisher('/rg2_finger_joint1_position_controller/command', Float64, queue_size=10)
        self.pub_finger2 = rospy.Publisher('/rg2_finger_joint2_position_controller/command', Float64, queue_size=10)
        
        # Warte, bis die Publisher verbunden sind
        rospy.sleep(2)
        rospy.loginfo("Controller initialisiert")
    
    def move_joints(self, joint_positions, wait_time=5.0):
        """Bewege die Gelenke direkt zu den angegebenen Positionen"""
        if len(joint_positions) != len(self.joint_names):
            rospy.logerr(f"Falsche Anzahl an Gelenkpositionen: {len(joint_positions)} statt {len(self.joint_names)}")
            return False
        
        rospy.loginfo(f"Bewege Gelenke zu: {joint_positions}")
        
        # Sende Positionen an die Controller
        for i, joint_name in enumerate(self.joint_names):
            self.joint_publishers[joint_name].publish(Float64(joint_positions[i]))
        
        # Warte, bis die Bewegung abgeschlossen ist
        rospy.sleep(wait_time)
        return True
    
    def open_gripper(self, wait_time=1.0):
        """Öffne den Greifer"""
        rospy.loginfo("Öffne den Greifer")
        self.pub_finger1.publish(Float64(1.0))
        self.pub_finger2.publish(Float64(1.0))
        rospy.sleep(wait_time)
    
    def close_gripper(self, wait_time=1.0):
        """Schließe den Greifer"""
        rospy.loginfo("Schließe den Greifer")
        self.pub_finger1.publish(Float64(0.5))
        self.pub_finger2.publish(Float64(0.5))
        rospy.sleep(wait_time)

def pick_ball():
    # Erstelle den Controller
    controller = DirectRobotController()
    
    # Öffne den Greifer
    controller.open_gripper()
    
    # Bewege den Roboter in eine sichere Ausgangsposition
    rospy.loginfo("Bewege zur Ausgangsposition")
    home_position = [0, 0, 0, 0, 0, 0]  # Einfache Position mit minimalen Bewegungen
    controller.move_joints(home_position, wait_time=1.0)
    
    # Bewege den Roboter zur Greifposition
    rospy.loginfo("Bewege zur Greifposition")
    grasp_position = [0, -0.15, -2.08, -0.15, 1.5, 1.5]  # Leicht nach vorne gebeugt
    controller.move_joints(grasp_position, wait_time=1.0)
    
    # Schließe den Greifer
    controller.close_gripper(wait_time=1.0)

    # Bewege den Roboter in eine sichere Ausgangsposition für den Wurf
    #rospy.loginfo("Bewege zur Wurf-Ausgangsposition")
    #throw_start_position = [-1.5, 0, 0, 0, 0, 1] # Einfache Position mit minimalen Bewegungen
    #controller.move_joints(throw_start_position, wait_time=4.0)

    # Führe die Wurfbewegung aus
    rospy.loginfo("Führe Wurfbewegung aus")
    # Schnell nach vorne schwingen für den Wurf
    throw_motion_position = [-1.5, 1.3, 0, 0, 0, 0]
    controller.move_joints(throw_motion_position, wait_time=1.5)  # Schnellere Bewegung für den Wurf

    # Führe die Wurfbewegung aus
    rospy.loginfo("Führe Wurfbewegung aus")
    # Schnell nach vorne schwingen für den Wurf
    throw_motion_position = [-1.5, -0.6, 0, 0, 0, 0]
    controller.move_joints(throw_motion_position, wait_time=0)  # Schnellere Bewegung für den Wurf


    # Öffne den Greifer während der Wurfbewegung, um den Ball freizugeben
    controller.open_gripper(wait_time=4)  # Schnelles Öffnen

    # Vollende die Wurfbewegung
    #follow_through_position = [-1.5, -1.0, -1.3, -0.8, 0, 1]
    #controller.move_joints(follow_through_position, wait_time=0.5)

    # Bewege zurück zur Ausgangsposition
    rospy.loginfo("Bewege zurück zur Ausgangsposition")
    controller.move_joints(home_position, wait_time=1.0)

    # Greifer ist bereits geöffnet
    rospy.loginfo("Aufgabe abgeschlossen")

if __name__ == '__main__':
    try:
        pick_ball()
    except rospy.ROSInterruptException:
        pass 