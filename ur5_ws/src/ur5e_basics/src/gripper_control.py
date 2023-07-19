#!/usr/bin/env python
"""
Sample code which ilustrates the methods needed to control
the robotiq gripper using the python interface
"""
import rospy
import robotiq_2f_gripper_control.robotiq_2f_gripper_ctrl as rg

#Cons
FULL_OPEN = 0.087
FULL_CLOSE = 0

#Init node
rospy.init_node('gripperControl')

#Create gripper object
gripper = rg.RobotiqCGripper()

#Connect and reset
gripper.wait_for_connection()
if gripper.is_reset():
    gripper.reset()
    gripper.activate()
print(gripper.close(block=True))

#Move gripper
gripper.goto(pos= FULL_OPEN, vel = 0.100, force = 100)


