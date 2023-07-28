#!/usr/bin/env python
"""
Sample code which enables the return of coordenates
published by the robotic arm
"""
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def readCurrentPose(moveGroup,displayMode = "v"):

    if displayMode == "v":
        current_pose = moveGroup.get_current_pose().pose
    elif displayMode == "a":
        current_pose = moveGroup.get_current_pose().pose
        current_pose = [[current_pose.position.x,current_pose.position.y,current_pose.position.z],[current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w]]
    return current_pose


def moveToPose(moveGroup, targetPose):
    moveGroup.set_pose_target(targetPose)
    plan = moveGroup.go()

def constructPose(pointsArray):
    print(">> ", pointsArray)
    newPose = Pose()
    newPose.position.x = pointsArray[0][0]
    newPose.position.y = pointsArray[0][1]
    newPose.position.z = pointsArray[0][2]
    
    newPose.orientation.x = pointsArray[1][0]
    newPose.orientation.y = pointsArray[1][1]
    newPose.orientation.z = pointsArray[1][2]
    newPose.orientation.w = pointsArray[1][3]
    return newPose
    
    