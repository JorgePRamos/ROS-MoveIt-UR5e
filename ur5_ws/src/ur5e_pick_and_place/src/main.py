#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import json
import robot_current_pos as rcp
import gripper_control as gc
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
        userInput = input("Load plan?")
        userLoop = ""
        outPutPlan = {}
        if userInput == "y":
            with open('plan.json', 'r') as openfile:
                # Reading from json file
                outPutPlan = json.load(openfile)
        
        while(userLoop == ""):
            
            newPos= rcp.readCurrentPose(move_group,"a")
            posName = input("New pose name: ")
            
            outPutPlan[posName] = [newPos[0],newPos[1]]

            print("---------------------------------------------")
            userLoop = input("Press enter to continue any other to exit:  ")
 
        with open("plan.json", "w") as outfile:
            json.dump(outPutPlan, outfile)
    else:
        print("<=  Execution mode  =>")
        movePlan = {}
        # Opening JSON file
        with open('plan.json', 'r') as openfile:
        
            # Reading from json file
            movePlan = json.load(openfile)

        
        #Execute move plan
        for point in movePlan:
            gripper = gc.activateGipper()
            
            #Open gripper
            gripper.goto(pos= gc.FULL_CLOSE, vel = 0.100, force = 100)
            #rcp.moveToPose(move_group,rcp.constructPose(movePlan[point]))
            if point == "3":
                #Tile aproximation step
                gripper = gc.activateGipper()
                
                #Open gripper
                gripper.goto(pos= gc.FULL_CLOSE, vel = 0.100, force = 100)

                
    
    




