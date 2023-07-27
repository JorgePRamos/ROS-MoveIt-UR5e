#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import robot_current_pos as rcp

import scene_manager as sm
#Iniciate connection 
#Load scene
#Load menu
"""
Concept
Menu
    - Plan
        Allow user to get poses in a dic
        Save them to memory in json
    - Run
        Load plan from Json
        Run

Think how to manage scene object 
        
"""

if __name__ == '__main__':
    referenceFrameId = "frame_base_link"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5e_rospy_controller', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #Set MoveIt arm group
    group_name = "ur5e_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

    sm.loadScene("mainScene",scene)
    #Remove object
    """
    scene.remove_attached_object(referenceFrameId,"tile_rack")

    scene.remove_world_object("tile_rack")
    
    """
    userInput = input("Introduce < P > for planning mode or < E > for execution mode:  ")
    
    if str.upper(userInput) == "P":
        print("<=  Planning mode  =>")
        userLoop = ""
        while(userLoop == ""):
            cp = rcp.readCurrentPose(move_group,"a")
            print("Position: ")
            print(cp[0])
            print("===")
            print("Orientation: ")
            print(cp[1])

            print("---------------------------------------------")
            userLoop = input("Press enter to continue any other to exit:  ")

    
    
    



