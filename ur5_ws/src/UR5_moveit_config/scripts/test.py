#!/usr/bin/env python

from operator import truediv
import sys
import re
import os.path
import os
import copy
import rospy
from math import pi
import numpy as np
from datetime import datetime
import time
from enum import Enum
import json
from collections import OrderedDict
import threading

# ROS libraries
import moveit_commander
from std_msgs.msg import String
import moveit_msgs.msg
from moveit_msgs.msg import ExecuteTrajectoryActionFeedback
from geometry_msgs.msg import Quaternion, Pose, Point
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest
from sensor_msgs.msg import Illuminance
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from PostaDatabaseClient import PostaDatabase, PostaSample, PostaSampleTray
from PostaClient import PostaClient


class GripperController:
    def __init__(self):

        # ROS Robotiq Gripper NODES
        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', 
                                            outputMsg.Robotiq2FGripper_robot_output,
                                            queue_size=10)
        
        robotiq_command = outputMsg.Robotiq2FGripper_robot_output()

        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.robotiq_status_listener_callback)

        self.gripper_in_position = False


    def robotiq_status_listener_callback(self, status):

        self.gripper_last_update = datetime.now()

        if(status.gACT==0):
            self.gripper_active = False
        else:
            if(status.gSTA==3):
                self.gripper_active = True
            else:
                self.gripper_active = False

        if(status.gOBJ == 3):
            self.gripper_in_position = True
            print('STAUTUS: ', g.gripper_in_position)
            rospy.sleep(1) 

        else:
            self.gripper_in_position = False

        self.gripper_position_value = status.gPR
        self.gripper_position_value_mm = 86.6 - 0.38*self.gripper_position_value

if __name__ == '__main__':

    g = GripperController()

    while True:
        print(g.gripper_in_position)
        rospy.sleep(1) 
    