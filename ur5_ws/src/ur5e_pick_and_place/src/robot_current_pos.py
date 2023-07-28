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


def moveToPose(moveGroup, targetPose,velocityScalingFactor = 0.5):
    moveGroup.set_pose_target(targetPose)
    moveGroup.set_max_velocity_scaling_factor(velocityScalingFactor)
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
    
    
def executePath(moveGRoup,targetPose, speed_scaling, max_retrials = 3):
        (plan, fraction) = moveGRoup.compute_cartesian_path(targetPose,0.005,0.0)
        # retime trajectory
        state = moveGRoup.robot.get_current_state()
        plan = moveGRoup.retime_trajectory(state,plan,speed_scaling)
        cnt = 0
        while cnt<max_retrials:
            cnt += 1
            response = moveGRoup.execute(plan, wait=True)
            if(response):
                return True
            rospy.sleep(0.5)