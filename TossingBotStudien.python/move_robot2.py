#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

def main():
    rospy.init_node('move_robot', anonymous=True)

    # Initialisiere MoveIt-Komponenten
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "manipulator"
    move_group = MoveGroupCommander(group_name)

    # Zielpose definieren
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4  # Anpassen an den Arbeitsbereich
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    # Bewegung planen und ausführen
    rospy.loginfo("Planung der Bewegung...")
    plan = move_group.plan()

    if plan and plan[0]:
        rospy.loginfo("Bewegung ausführen...")
        success = move_group.execute(plan[1], wait=True)
        if success:
            rospy.loginfo("Bewegung erfolgreich ausgeführt!")
        else:
            rospy.logwarn("Bewegung fehlgeschlagen!")
    else:
        rospy.logerr("Planung fehlgeschlagen!")

    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
