#!/usr/bin/env python
import rospy
import robotiq_2f_gripper_control.robotiq_2f_gripper_ctrl as rg
#Cons
FULL_OPEN = 0.087
FULL_CLOSE = 0

def activateGipper():
    #Init node
    #Create gripper object
    gripper = rg.RobotiqCGripper()

    #Connect and reset
    gripper.wait_for_connection()
    if gripper.is_reset():
        gripper.reset()
        gripper.activate()
    print(gripper.close(block=True))
    
    return gripper