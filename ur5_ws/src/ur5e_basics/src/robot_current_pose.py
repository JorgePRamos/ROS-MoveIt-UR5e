#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs


def readCurrentPose(moveGroup):

    current_pose = moveGroup.get_current_pose().pose
    return current_pose

def arrayToPose(targetArray):
    resultingPose = geometry_msgs.msg.Pose()

    """
    print("targetArray[0]:  ",targetArray[0])
    resultingPose.position.x = targetArray[0]
    resultingPose.position.y = targetArray[1]
    resultingPose.position.z = targetArray[2]
    resultingPose.orientation.x = targetArray[3]
    resultingPose.orientation.y = targetArray[4]
    resultingPose.orientation.z = targetArray[5]
    resultingPose.orientation.w = targetArray[6]
    return resultingPose"""

def moveToPose(moveGroup, targetPose):
    moveGroup.set_pose_target(targetPose)
    plan = moveGroup.go()



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5e_current_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    #Set MoveIt arm group
    group_name = "ur5e_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


    #Get Origin pose
    userInput = input("Set target pose and press any key....")
    targetPose = readCurrentPose(move_group)
    print(targetPose)

    #Get Target pose
    userInput = input("Set origin pose and press any key....")
    originPose = readCurrentPose(move_group)
    print(originPose)
    #Start movement
    userInput = input("Press any key to start...")

    #Movement
    #moveToPose(move_group,targetPose)
