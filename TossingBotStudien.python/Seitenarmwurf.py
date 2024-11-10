import rospy
from threading import Thread
from time import sleep
import actionlib
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from franka_gripper.msg import GraspAction, GraspGoal, MoveGoal, MoveAction
from move_group_python_interface_tutorial import MoveGroupPythonInterfaceTutorial

def spawnObject(model_name, pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = pose[0]
        state.pose.position.y = pose[1]
        state.pose.position.z = pose[2]
        set_model_state(state)
        rospy.loginfo("Spawned object '{}' successfully.".format(model_name))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))

def controlGripper(width, force):
    if force == 0:
        rospy.loginfo("Opening gripper")
        client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        rospy.loginfo("Waiting for the move action server to start...")
        client.wait_for_server()
        rospy.loginfo("Move action server found.")
        goal = MoveGoal(width=width, speed=1.0)
    else:
        rospy.loginfo("Closing gripper")
        client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        rospy.loginfo("Waiting for the grasp action server to start...")
        client.wait_for_server()
        rospy.loginfo("Grasp action server found.")
        goal = GraspGoal(width=width, speed=1.0, force=force)
        goal.epsilon.inner = 0.001
        goal.epsilon.outer = 0.001

    rospy.loginfo("Sending goal to action server...")
    client.send_goal(goal)

    client.wait_for_result()
    result = client.get_result()
    print(result)
    rospy.loginfo("Result: Success - %s", result.success if result else "No result received")

def throwMovement(robot):#130
    robot.go_to_joint_state(20, -90, 90, -4, 0, 180, 45)


def throwOpen(): #1.4
    sleep(1.4)
    controlGripper(0.08, 0)

def throw(robot):
    movement = Thread(target=throwMovement, args=(robot,))
    open = Thread(target=throwOpen)

    movement.start()
    open.start()
    
    movement.join()
    open.join()

def main():
    try:
        # Initialisiere den ROS-Knoten für den Greifer
        rospy.init_node('throwController')

        # Erstellen eines MoveGroup-Objekts für den Roboter
        robot = MoveGroupPythonInterfaceTutorial()

        # Bewegungen des Roboters und Steuerung des Greifers
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)
        controlGripper(0.08, 0)
        robot.go_to_joint_state(0, 19, 0, -153, 0, 171, 45)
        controlGripper(0.06, 10)
        # Objekt anheben
        robot.go_to_joint_state(0, -10, 0, -150, 0, 130, 45)

        # Ausholen
        robot.go_to_joint_state(-110, -10, 0, -150, 0, 130, 45)


        # Wurf
        throw(robot)

        # Warten bevor Objekt spawnen soll
        input()

        # Spawne Objekt
        model_name = "cube"
        pose = [0.4, 0, 0.4190880]
        spawnObject(model_name, pose)

    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
