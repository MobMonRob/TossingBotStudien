#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Float64
import time

def gripper_control():
    # Initialize the node
    rospy.init_node('gripper_control', anonymous=True)
    
    # Create publishers for both finger joints
    pub_finger1 = rospy.Publisher('/rg2_finger_joint1_position_controller/command', Float64, queue_size=10)
    pub_finger2 = rospy.Publisher('/rg2_finger_joint2_position_controller/command', Float64, queue_size=10)
    
    # Wait for publishers to connect
    rospy.sleep(2)
    
    rate = rospy.Rate(10)  # 10hz
    
    try:
        while not rospy.is_shutdown():
            # Ensure initial position
            rospy.loginfo("Setting initial position")
            pub_finger1.publish(Float64(0.0))
            pub_finger2.publish(Float64(0.0))
            rospy.sleep(3)
            
            # Open gripper (fully)
            rospy.loginfo("Opening gripper fully")
            for i in range(11):
                position = i * 0.1
                pub_finger1.publish(Float64(position))
                pub_finger2.publish(Float64(position))  # Auch den zweiten Finger steuern
                rospy.sleep(0.1)
            rospy.sleep(2)
            
            # Close gripper
            rospy.loginfo("Closing gripper")
            for i in range(11):
                position = 1.0 - (i * 0.1)
                pub_finger1.publish(Float64(position))
                pub_finger2.publish(Float64(position))  # Auch den zweiten Finger steuern
                rospy.sleep(0.1)
            rospy.sleep(2)
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        gripper_control()
    except rospy.ROSInterruptException:
        pass 