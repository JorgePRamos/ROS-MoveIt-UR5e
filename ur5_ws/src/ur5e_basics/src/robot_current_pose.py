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


def readCurrentPose(moveGroup):

    current_pose = moveGroup.get_current_pose().pose
    return current_pose


def moveToPose(moveGroup, targetPose):
    moveGroup.set_pose_target(targetPose)
    plan = moveGroup.go()

