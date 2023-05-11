#!/usr/bin/env python

import tkinter as tk
import time 
from tkinter import messagebox

import rospy
from std_msgs.msg import String
from enum import Enum
import json
from collections import OrderedDict
import threading
import random
from enum import Enum


class PostaCommands(Enum):
    """
    Set of all possible commands
    """
    Reset = 1
    Ping = 2
    OpenGripper = 3
    CloseGripper = 4
    SetGripperApertureMM = 5
    GoToRestPose = 6
    OpenDrawer = 7
    CloseDrawer = 8
    GoToDrawerImaging = 9
    PickNextPouch = 10
    PutPouchOnScale = 11
    PickPouchFromScale = 12
    SetScaleToZero = 13
    PutPouchOnStrengthPlate = 14
    PutPaperSheetOnStrength = 15
    DisposePaperSheetFromStrength = 16
    GoFromStrengthToImaging = 17
    StartStrengthTest = 18
    StopCurrentRobotTrajectory = 19


class SUDPostaControllerV2: 
    
    def __init__(self): 
        rospy.loginfo('SUD Posta Controller Simulator Initialized')

        # State Variables
        self.current_drawer_number = 0
        self.current_drawer_isopen = False
        self.robot_motion_status = 'idle'
        self.stop_robot_motion_flag = False
        self.gripper_aperture_forPouch = 23
        self.number_of_pouches_detected = 0
        self.pouch_in_gripper = False
        self.posta_client_online = False
        self.mark10_instrument_ready = True
        self.mark10_test_completed = True
        self.mark10_plate_distance = -1
        self.mark10_plate_moving_up = False
        self.posta_app_status = ''
        self.moveit_trajectory_execution_status = ''
        self.local_status_file = 'sud_posta_status.json'

        # Gripper Variables
        self.gripper_active = False
        self.gripper_in_position = False
        self.gripper_position_value = 0
        self.gripper_last_header = None

        # Balance Variables
        self.ScaleWeight = 0
        self.ScaleWeightTimestamp = time.time()


        # Controller NODE
        rospy.init_node('sud_posta_controller', anonymous=True)

        # TOPIC Input commands
        self.input_commands_topic = "/sud_posta_commands"
        self.latest_command_string = ""
        self.latest_command = None
        self.latest_command_parameter_1 = None
        self.latest_command_parameter_2 = None

        rospy.Subscriber(self.input_commands_topic, String, self.input_commands_callback)

        # PUBLISHER of internal status
        self.sud_posta_state_publisher = rospy.Publisher("sud_posta_state", String, queue_size=10)

        # PUBLISHER of error messages
        self.error_publisher = rospy.Publisher("sud_posta_errors", String, queue_size=10)

        # COMMANDS QUEUE
        self.commands_queue = []


    def input_commands_callback(self, data):

        self.latest_command_string = str(data.data)

        valid_command, response, command, p1, p2 = self.parse_command(self.latest_command_string)

        if (valid_command):

            if command == PostaCommands.StopCurrentRobotTrajectory:
                self.stop_robot_motion_flag = True
                rospy.loginfo('--> STOP COMMAND RECEIVED')

            self.commands_queue.append(self.latest_command_string)
            self.latest_command = command
            self.latest_command_parameter_1 = p1
            self.latest_command_parameter_2 = p2
        else:                
            rospy.logerr('Command received with Error: {0}'.format(response))
    

    def parse_command(self, command_string):
        if (len(command_string) > 0):

            values = command_string.split('|')

            if (values[0] in PostaCommands._member_names_):

                command = PostaCommands[values[0]]
                parameter1 = None
                parameter2 = None

                if (len(values) > 1):
                    parameter1 = values[1]

                if (len(values) > 2):
                    parameter2 = values[2]

                return True, '', command, parameter1, parameter2

            else:
                return False, 'Received command not VALID: {0}'.format(self.latest_command_string), None, None, None
        else:
            return False, 'Received command has zero length', None, None, None
    

    def SimulateRobotMotion(self, duration):
                
        start_t = time.time()
        current_t = time.time()
        
        self.robot_motion_status = 'moving'
        self.moveit_trajectory_execution_status = 'MONITOR'

        # Helps to simulate the robot motion
        while current_t-start_t < duration: 
            
            current_t = time.time()

            if self.stop_robot_motion_flag:
                rospy.loginfo('STOP INTERRUPT received in the low level motion function')
                self.robot_motion_status = 'stopped'
                self.stop_robot_motion_flag = False

                return False
            
            rospy.sleep(0.3)

        self.robot_motion_status = 'idle'
        self.moveit_trajectory_execution_status = ''

        return True

        
    # --- THREAD ----
    def StatusPublishingThread(self):
        while not rospy.is_shutdown():
            # publish the latest status
            self.sud_posta_state_publisher.publish(self.generate_status_json())
            rospy.sleep(0.5)


    # We need a loop that executes all what's inside the command queue
    def ControllerStart(self):
        
        # Status Thread
        self.status_thread = threading.Thread(target=self.StatusPublishingThread)
        self.status_thread.start()


        while not rospy.is_shutdown():

            # If there are commands in the queue execute the first one

            if (len(self.commands_queue) > 0):
                command_string = self.commands_queue.pop(0)
                valid_command, response, command, p1, p2 = self.parse_command(command_string)

                if valid_command:

                    if (command == PostaCommands.OpenGripper):
                        rospy.sleep(2)
                        self.gripper_position_value = 0
                        rospy.loginfo('--> OpenGripper command Executed')
                    
                    elif (command == PostaCommands.CloseGripper):
                        rospy.sleep(2)
                        self.gripper_position_value = 255
                        rospy.loginfo('--> CloseGripper command Executed')

                    elif (command == PostaCommands.OpenDrawer):
                        
                        target_drawer_number = int(p1)

                        # If a drawer is open
                        if (self.current_drawer_isopen):
                            # if that open drawer is the one we want to open
                            # This really never happens when using the GUI
                            if (self.current_drawer_number == target_drawer_number):
                                # drawer already open
                                rospy.loginfo("Drawer[{0}] is already open.".format(target_drawer_number))
                            else:
                                err_str = 'ERROR|Drawer {0} cannot be opened while drawer {1} is open, close this one first'.format(
                                        target_drawer_number, self.current_drawer_number)
                                rospy.loginfo(err_str)
                                self.error_publisher.publish(err_str)
                        
                        # if no drawer is open
                        else:
                            self.current_drawer_number = target_drawer_number

                            rospy.loginfo('No currently opened drawer, proceed to open drawer {0}...'.format(
                                self.current_drawer_number))
                            
                            action = self.SimulateRobotMotion(5)

                            if action:
                                self.current_drawer_isopen = True
                                self.number_of_pouches_detected = random.randrange(15)

                                rospy.loginfo('--> Opening drawer {0} COMPLETED'.format(self.current_drawer_number))
                            
                            else: 
                                self.error_publisher.publish('INFO|Execution Stopped')

                    elif (command == PostaCommands.CloseDrawer):

                        target_drawer_number = int(p1)
                        
                        if (self.current_drawer_isopen):
                            self.current_drawer_number = target_drawer_number
                            rospy.loginfo('Drawer {0} is currently open, closing it down...')

                            action = self.SimulateRobotMotion(5)

                            if action:
                                self.current_drawer_isopen = False
                                self.number_of_pouches_detected = 0

                                rospy.loginfo('--> Closing Drawer {0} COMPLETED'.format(self.current_drawer_number))
                            
                            else: 
                                self.error_publisher.publish('ERROR|Execution Stopped')
                            
                        else:
                            # no open drawer, nothing to closeScaleWeightTimestamp
                            # This never happens when using the GUI
                            self.current_drawer_number = target_drawer_number
                            rospy.loginfo('Drawer {0} is already closed, no action'.format(self.current_drawer_number))

                    elif (command == PostaCommands.GoToDrawerImaging):

                        target_drawer_number = int(p1)

                        if (self.current_drawer_number == target_drawer_number) and (self.current_drawer_isopen):
                            rospy.loginfo('Going to Drawer {0} imaging position'.format(self.current_drawer_number))
                            self.SimulateRobotMotion(5)
                            rospy.loginfo('--> Going to drawer imaging COMPLETED')
                        else:
                            warn = 'WARNING|The target drawer {0} is not open, I am not going to that imaging position'.format(target_drawer_number)
                            rospy.logwarn(warn)
                            self.error_publisher.publish(warn)
                    
                    elif (command == PostaCommands.SetGripperApertureMM):

                        p1 = int(p1)

                       # Simulates the waiting time 
                        delta = abs(self.gripper_position_value - p1)
                        t = 2*delta / 255
                        rospy.sleep(t)

                        self.gripper_position_value = p1
                        rospy.loginfo('--> SetGripperValue command Executed')

                    elif (command == PostaCommands.SetScaleToZero):
                        rospy.sleep(0.5)
                        self.ScaleWeight = 0.0
                        rospy.loginfo('--> Scale setted to Zero')

                    elif (command == PostaCommands.PutPouchOnScale):
                        
                        # Simulating Movement
                        self.SimulateRobotMotion(5)

                        # Opening Gripper
                        self.gripper_position_value = 60
                        self.pouch_in_gripper = False

                        # Simulating Weight
                        rospy.sleep(1)
                        weight = round(random.uniform(33.33, 66.66), 2)
                        self.ScaleWeight = weight
                        self.ScaleWeightTimestamp = time.time()

                        rospy.loginfo('--> Pouch putted on scale')

                    elif (command == PostaCommands.PickPouchFromScale):

                        # Simulating Movement
                        self.SimulateRobotMotion(2)

                        # Closing Gripper
                        rospy.sleep(1)
                        self.gripper_position_value = self.gripper_aperture_forPouch
                        self.pouch_in_gripper = True

                        # Simulating Weight
                        self.SimulateRobotMotion(3)
                        
                        rospy.loginfo('--> Pouch picked on scale')
                                                
                    elif (command == PostaCommands.PutPouchOnStrengthPlate):
                        if self.pouch_in_gripper:
                            rospy.loginfo('Going to put pouch on strength plate')
                            
                            self.SimulateRobotMotion(5)

                            self.gripper_position_value = 45
                            self.pouch_in_gripper = False

                            rospy.loginfo('--> pouch on strength plate completed')
                            
                        else:
                            err = 'ERROR|There is no pouch in gripper, nothing to put on strength plate'
                            rospy.logerr(err)
                            self.error_publisher.publish(err)
                            
                    elif (command == PostaCommands.PutPaperSheetOnStrength):
                        if(not self.pouch_in_gripper):
                            rospy.loginfo('Going to put put paper sheet on strength plate')

                            # depending on where the robot is, bring him closer
                            self.SimulateRobotMotion(2)
                            
                            # Open the gripper
                            self.gripper_position_value = 60
                            rospy.sleep(1)

                            # Go to the pickup position
                            self.SimulateRobotMotion(2)
                            
                            # Close the gripper and turn on vacuum
                            self.gripper_position_value = 0
                            rospy.sleep(1)

                            # Put the paper on the tray
                            self.SimulateRobotMotion(2)
                            
                            # Put back the vacuum holder
                            self.SimulateRobotMotion(2)
                            
                            # Open the gripper with the vacuum on to stabilize it
                            self.gripper_position_value = 60
                            rospy.sleep(0.1)

                            # Move back to 
                            self.SimulateRobotMotion(2)

                            rospy.loginfo('--> put paper sheet completed')

                        else:
                            err = 'ERROR|There is currently a pouch in the gripper, cannot put a paper sheet' 
                            rospy.logerr(err)
                            self.error_publisher.publish(err)
                    
                    elif (command == PostaCommands.DisposePaperSheetFromStrength):
                        if(not self.pouch_in_gripper):
                            rospy.loginfo('Going to dispose paper sheets')

                            # approach the dispose point - reworked on Jan23 with new plate
                            self.SimulateRobotMotion(2)

                            # Open the gripper
                            self.gripper_position_value = 60
                            rospy.sleep(0.3)

                            # go to the dispose point
                            self.SimulateRobotMotion(2)

                            # Close the gripper
                            self.gripper_position_value = 0
                            rospy.sleep(0.3)

                            # lift the papers to dispose
                            self.SimulateRobotMotion(2)
                            # Open the gripper
                            self.gripper_position_value = 60
                            rospy.sleep(0.1) # wait for the paper to fall off

                            # Return to imaging
                            self.SimulateRobotMotion(2)

                            rospy.loginfo('--> paper sheets disposing completed')

                        else:
                            err = 'ERROR|There is currently a pouch in the gripper, cannot dispose a paper sheet'
                            rospy.logerr(err)
                            self.error_publisher.publish(err)

                    elif (command == PostaCommands.StartStrengthTest):
                        if(self.mark10_instrument_ready and self.mark10_test_completed):
                            rospy.loginfo('Starting Mark10 Strength Test')
                            self.mark10_instrument_ready = False
                            self.mark10_test_completed = False
                            rospy.sleep(5)
                            rospy.loginfo('--> Strength Test Completed')
                            self.mark10_plate_moving_up = True
                            self.mark10_instrument_ready = True
                            self.mark10_test_completed = True
                        else:
                            if not self.mark10_instrument_ready: 
                                err = 'ERROR|Cannot start strength test, the instrument is not ready'
                            else:
                                err = 'Cannot start strength test, the test is not completed'
                            rospy.logerr(err) 
                            self.error_publisher.publish(err)

                    elif (command == PostaCommands.GoToRestPose):
                        rospy.loginfo('Going to rest pose')
                        # approach the dispose point - reworked on Jan23 with new plate
                        self.SimulateRobotMotion(2)

                    elif (command == PostaCommands.PickNextPouch):
                        
                        # Open Gripper
                        self.gripper_position_value = 200
                        # Move
                        self.SimulateRobotMotion(2)
                        # Close Gripper
                        self.gripper_position_value = self.gripper_aperture_forPouch
                        self.pouch_in_gripper = True
                    
                    elif (command == PostaCommands.GoFromStrengthToImaging):
                        self.SimulateRobotMotion(2)

            rospy.sleep(0.5)


    def generate_status_json(self):

        self.gripper_position_value_mm = 86.6 - 0.38 * self.gripper_position_value

        status = OrderedDict(
            timestamp=time.time(),
            commands_in_queue=len(self.commands_queue),
            gripper_active=self.gripper_active,
            gripper_in_position=self.gripper_in_position,
            gripper_aperture=self.gripper_position_value,
            gripper_aperture_mm=self.gripper_position_value_mm
        )

        status['robot'] = self.robot_motion_status
        status['moveit_trajectory_status'] = self.moveit_trajectory_execution_status

        for i in range(0, 6):
            dNum = i + 1
            dNumString = 'drawer_{0}'.format(dNum)
            if (self.current_drawer_number == dNum):
                if (self.current_drawer_isopen):
                    status[dNumString] = 'open'
                else:
                    status[dNumString] = 'closed'
            else:
                status[dNumString] = 'closed'

        status["number_of_detected_pouches"] = self.number_of_pouches_detected
        status['pouch_in_gripper'] = self.pouch_in_gripper

        status['scale_weight'] = self.ScaleWeight
        status['scale_timestamp'] = self.ScaleWeightTimestamp

        status['posta_client_online'] = self.posta_client_online
        status['mark10_instrument_ready'] = self.mark10_instrument_ready
        status['mark10_test_completed'] = self.mark10_test_completed
        status['mark10_plate_distance'] = self.mark10_plate_distance
        status['mark10_plate_moving_up'] = self.mark10_plate_moving_up

        status_json = json.dumps(status)

        # Save to local file
        with open(self.local_status_file, 'w') as file:
            file.write(status_json)

        return status_json
        

if __name__ == '__main__':
    try: 
        a = SUDPostaControllerV2()
        a.ControllerStart()

    except rospy.ROSInterruptException: 
        pass