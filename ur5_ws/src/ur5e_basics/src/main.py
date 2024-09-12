#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

import robot_current_pose as rcp
import scene_builder as sb

if __name__ == '__main__':
    referenceFrameId = "frame_base_link"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5e_rospy_controller', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #Set MoveIt arm group
    group_name = "ur5e_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
    
    #Initialize scene
    #sb.newObjectToScene(referenceFrameId,scene,"tile_holder",[0.34,0.34,0.29],[0,0,0.146])

    #Get current pose
    currentPose = rcp.readCurrentPose(move_group)
    print(">> Current pose: ",currentPose)