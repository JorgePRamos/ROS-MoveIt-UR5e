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
import time
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
    #Gripper init
    gripper = gc.activateGipper()
    gripper.goto(pos= gc.FULL_OPEN, vel = 0.100, force = 100)
    
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
    touch_links = robot.get_link_names(group="gripper")

    
    sm.newObjectToScene(referenceFrameId,scene,"tile",[0.29,0.29,0.01],[-0.12,-0.36,0.37])
    scene.attach_box("tool0","tile",touch_links= touch_links)
    
    userInput = input("Introduce < P > for planning mode or < E > for execution mode:  ")
    
    if str.upper(userInput) == "P":
        print("<=  Planning mode  =>")
        userInput = input("Load plan? ")
        userLoop = ""
        outPutPlan = {}
        if userInput == "y":
            with open('plan.json', 'r') as openfile:
                # Reading from json file
                outPutPlan = json.load(openfile)
        
        while(userLoop == ""):
            
            
            posName = input("New pose name: ")
            newPos= rcp.readCurrentPose(move_group,"a")
            
            outPutPlan[posName] = [newPos[0],newPos[1]]

            print("---------------------------------------------")
            userLoop = input("Press enter to continue any other to exit:  ")
 
        with open("plan.json", "w") as outfile:
            json.dump(outPutPlan, outfile)
    
    elif str.upper(userInput) == "T":
        print("<=  Test mode  =>")
        loop = ""
        while(loop == ""):
            gripper.goto(pos= gc.FULL_OPEN, vel = 0.100, force = 100)
            
            
       
            userInput = input("Press enter to close gripper")
            #self.cur_status.gOBJ == 1 or self.cur_status.gOBJ == 2
            gripper.goto(pos= gc.FULL_CLOSE, vel = 0.100, force = 100)

        
            loop = input("Press enter to run test again Other to exit...")

    
    else:
        print("<=  Execution mode  =>")
        movePlan = {}
        #Connect to gripper

        
        # Opening JSON file
        with open('plan.json', 'r') as openfile:
        
            # Reading from json file
            movePlan = json.load(openfile)

        
        #Execute move plan
        
        
        for step in movePlan:
 
 
            if step == "o":
                print("Move to origin")
                rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                               
     
            if step == "3":
                #Tile aproximation step
                #Open gripper
                move_group.set_max_velocity_scaling_factor(0.05)
                #rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                waypoints = []
                waypoints.append(rcp.constructPose(movePlan[step]))
                rcp.executePath(move_group,waypoints,0.5)
                gripper.goto(pos= gc.FULL_OPEN, vel = 0.1, force = 100)

            elif step == "4":
                move_group.set_max_velocity_scaling_factor(0.05)
                #rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                waypoints = []
                waypoints.append(rcp.constructPose(movePlan[step]))
                rcp.executePath(move_group,waypoints,0.5)

                #Teporaly dactivate tile_rack collision
                scene.remove_attached_object(referenceFrameId,"tile_rack")
                scene.remove_world_object("tile_rack")
            
                #TODO Add tile to scene
            elif step == "6":
                move_group.set_max_velocity_scaling_factor(0.05)
                #rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                waypoints = []
                waypoints.append(rcp.constructPose(movePlan[step]))
                rcp.executePath(move_group,waypoints,0.5)

                #Grip tile
                gripper.goto(pos= gc.FULL_CLOSE, vel = 0.05, force = 80)
            elif step == "9":
                move_group.set_max_velocity_scaling_factor(0.05)
                #rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                waypoints = []
                waypoints.append(rcp.constructPose(movePlan[step]))
                rcp.executePath(move_group,waypoints,0.5)
                sm.loadScene("mainScene",scene)
            else:
                move_group.set_max_velocity_scaling_factor(0.05)
                #rcp.moveToPose(move_group,rcp.constructPose(movePlan[step]))
                waypoints = []
                waypoints.append(rcp.constructPose(movePlan[step]))
                rcp.executePath(move_group,waypoints,0.5)

                

                
                

                
    
    




