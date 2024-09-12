#!/usr/bin/env python
import rospy
import robotiq_2f_gripper_control.robotiq_2f_gripper_ctrl as rg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')

#Cons
FULL_OPEN = 0.087
FULL_CLOSE = 0

def activateGipper():
    #Connect to gripper
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size=20)
    #Reset    
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.1)
    print(">> Gripper reset")
    #Activate
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    pub.publish(command)
    rospy.sleep(0.1)
    print(">> Gripper activated")

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
