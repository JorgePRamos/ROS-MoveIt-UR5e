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

class PostaCommands(Enum):
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
    GetWeight = 20
    StopAll = 21

def ArrayToPose(v, delta_z = 0 ):
    #print 'Original waypoint array: ',v
    msg = Pose()
    msg.position.x = v[0]
    msg.position.y = v[1]
    msg.position.z = v[2] + delta_z
    msg.orientation.x = v[3]
    msg.orientation.y = v[4]
    msg.orientation.z = v[5]
    msg.orientation.w = v[6]
    return msg

def interpolate_in_array(a1,a2,xmin,xmax,x):
    # return linear interpolation in array
    aOut = copy.deepcopy(a1)
    s = (x-xmin)/(xmax-xmin) # normalized coordinated [0,1]
    for i in range(len(a1)):
        vMin = a1[i]
        vMax = a2[i]
        aOut[i] = vMin + s*(vMax-vMin)
    return aOut

def RadToDeg(v):
    return v * 180.0 / pi

def DegToRad(v):
    return v / 180.0 * pi



class Action:

    def __init__(self, agent, command, name, status, p1=None, p2=None):
        self.agent = agent
        self.command = command
        self.name = name
        self.status = status
        self.timestamp = time.time()
        self.p1 = p1
        self.p2 = p2


class SUDPostaControllerV2:

    def __init__(self):
        rospy.loginfo('SUD Posta Controller V2 INIT Started -------------------------------------')

        # Global parameters
        HOST = "192.168.1.7" # Posta Server Desktop PC
        PORT = 1978

        # Command Queue
        self.commands_queue = []
        self.actions_list = []

        self.SimulationMode = False

        # State Variables
        self.current_drawer_number = 0
        self.current_drawer_isopen = False
        self.robot_motion_status = 'idle'
        self.stop_robot_motion_flag = False
        self.gripper_aperture_forPouch = 23
        self.number_of_pouches_detected = 0
        self.pouch_in_gripper = False
        self.posta_client_online = False
        self.mark10_instrument_ready = False
        self.mark10_test_completed = False
        self.mark10_plate_distance = -1
        self.mark10_plate_moving_up = False
        self.posta_app_status = ''
        self.moveit_trajectory_execution_status = ''
        self.local_status_file = 'sud_posta_status.json'

        self.error_msg = None
        self.moveit_error_msg = None

        self.agents_status = {'robot': (None, None, None, None), 
                              'scale': (None, None, None, None), 
                              'mark10': (None, None, None, None)}
        
        # Controller NODE
        rospy.init_node('sud_posta_controller', anonymous=True)
        
        # TOPIC Input commands
        input_commands_topic = "/sud_posta_commands"
        rospy.Subscriber(input_commands_topic, String, self.input_commands_callback)
        self.latest_command_string = ""
        self.latest_command = None
        self.latest_command_parameter_1 = None
        self.latest_command_parameter_2 = None

        # PUBLISHER of internal status
        self.sud_posta_state_publisher = rospy.Publisher("sud_posta_state", String, queue_size=10)
        
        # ROS Robot NODES
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "ur5_arm" # this should be the same name used in RViz 
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        self.set_io_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        timeout = rospy.Duration(10)
        try:
            self.set_io_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach SetIO service. Make sure that the driver is actually running."
                " Msg: {}".format(err))
        self.robot_state = self.robot.get_current_state()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
            
        # ROS Robotiq Gripper NODES
        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', 
            outputMsg.Robotiq2FGripper_robot_output,
            queue_size=10)
        self.robotiq_command = outputMsg.Robotiq2FGripper_robot_output()

        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.robotiq_status_listener_callback)
        self.gripper_active = False
        self.gripper_in_position = False
        self.gripper_position_value = 0
        self.gripper_last_header = None

        # ROS Vision Topics
        bbox_topic = "/sud_detected_pouches_location"
        self.bounding_boxes = None
        self.bounding_boxes_header = None
        rospy.Subscriber(bbox_topic, BoundingBoxes, self.pouches_bbox_callback)

        darknet_bbox_topic = "/darknet_ros/bounding_boxes/"
        self.darknet_bounding_boxes = None
        self.darknet_header = None
        rospy.Subscriber(darknet_bbox_topic, BoundingBoxes, self.darknet_bbox_callback)

        rospy.Subscriber('MTScaleOutput', Illuminance, self.MTweight_callback )
        self.WeightCommandPublisher = rospy.Publisher('MTScaleCommands', String, queue_size=10)
        self.ScaleWeight = 0
        self.ScaleWeightTimeStamp = None

        # Subscribe to the feedback topic of execute trajectory
        rospy.Subscriber('/execute_trajectory/feedback', ExecuteTrajectoryActionFeedback, self.MoveitExecuteTrajectoryFeedbackCallback)

        self.PostaClient = PostaClient()

        # REFERENCE POSES
        self.distance_between_drawers = 0.115

        # BR corner of tray, used to convert to pickup coordinates from vision system
        self.d5_M0BR = [0.880048, 0.341341, 0.434975, -0.693886, 0.719742, 0.005294, 0.021572]
        self.pM0UL = (
            (self.d5_M0BR[0] * 1000.0) + 35,
            (self.d5_M0BR[1] * 1000.0) + 35
        )

        # positions for drawers opening and closing
        self.drawer_raised_position = [0.559734, -0.00245, 1.0618, -0.003348, -0.959885, -0.001748, 0.280369]
        self.drawer_half_cabinet = [0.492221, -0.00229, 0.62381, -0.004201, -0.939005, -0.000456, 0.343877]
        self.drawer6_handle_approach = [0.415184, 0.000814, 0.371352, -0.004018, -0.94294, 4.6e-05, 0.33294]
        self.drawer6_handle_engaged = [0.37, 0.000904, 0.313, -0.003933, -0.94291, 4.3e-05, 0.333023]
        self.drawer6_half_open = [0.674982, 0.002781, 0.316231, -0.003449, -0.943244, -0.002598, 0.332074]
        self.drawer6_half_open_up1 = [0.674837, 0.002211, 0.430359, -0.003724, -0.943191, -0.002088, 0.332224]
        self.drawer6_half_open_up2 = [0.595834, -9.9e-05, 0.42, -0.00393, -0.995784, 0.000587, 0.091647]
        self.drawer6_half_open_reengaged = [0.595831, -7e-06, 0.38239, -0.003999, -0.995782, 0.000489, 0.091662]
        self.drawer6_full_open = [0.930835, 0.000912, 0.36, 0.00427, 0.996572, 0.001443, 0.082601]
        self.drawer6_full_open_up = [0.930781, 0.000658, 0.424678, 0.004288, 0.996576, 0.001181, 0.08256]

        # Store positions for drawer 6
        self.drawer6_shoot_image = [0.568175, 0.001095, 1.09218, -0.704762, 0.709434, 0.002133, 0.003082]
        self.drawer6_pickup_center = [0.69133, 0.008065, 0.538871, -0.697368, 0.716553, 0.001512, 0.01512]
        self.drawer6_ul_plate = [0.852798, 0.280818, 0.350601, -0.693819, 0.719806, 0.005126, 0.021616]
        self.to_bin_1 = [0.488656, -0.206905, 1.220312, -0.705791, 0.708406, -0.000681, 0.004386]
        self.to_bin_2 = [0.385918, -0.309599, 1.324864, -0.706453, 0.707745, -0.002492, 0.00379]
        self.on_bin = [0.244225, -0.354504, 1.324834, -0.706391, 0.707807, -0.003797, 0.002513]

        self.to_strength_1 = [0.497763, -0.14425, 1.242678, 0.001415, 0.999989, 0.003113, 0.0033]
        self.to_strength_2 = [0.334348, -0.205948, 1.350463, -3.4e-05, 0.99999, -0.000238, 0.004496]
        self.to_strength_3 = [0.093791, -0.32839, 1.331235, -0.2323, -0.906055, 0.342027, 0.0901]
        self.to_strength_4 = [-0.013254, -0.264138, 1.298324, -0.620758, -0.639019, 0.320076, 0.322282]
        self.on_strength = [-0.186, -0.25074, 1.231288, -0.615637, -0.642769, 0.323707, 0.321018]

        self.vacuum_goingto_1 = [0.184223, -0.205344, 1.33083, 2e-05, 0.99999, -0.003204, 0.00318]
        self.vacuum_goingto_2 = [-0.0092, -0.161395, 1.391856, -0.698146, -0.696897, 0.119098, 0.112885]
        self.vacuum_above = [-0.090, 0.049765, 1.39199, -0.669726, -0.669794, 0.228473, 0.225041] # move x so that papers are push against cloth
        self.vacuum_pick = [-0.087, 0.05004, 1.331118, -0.670085, -0.669412, 0.228527, 0.225053] 
        self.vacuum_pick_down = [-0.086918, 0.05004, 1.329, -0.670085, -0.669412, 0.228527, 0.225053] 
        self.vacuum_abovePlate = [-0.081182, -0.259162, 1.389521, -0.673151, -0.669613, 0.224092, 0.219702]
        self.vacuum_abovePlate_pushOnColumn = [-0.08918, -0.259144, 1.389522, -0.67318, -0.669557, 0.224157, 0.219718]
        self.vacuum_onPlate = [-0.089487, -0.258831, 1.28519, -0.673805, -0.668888, 0.223834, 0.220166]
        self.vacuum_onPlate_withPouch = [-0.089439, -0.258889, 1.299725, -0.673717, -0.668985, 0.223891, 0.220084]
        
        self.vacuum_above_plastic = [-0.081165, -0.063093, 1.391242, -0.671119, -0.669434, 0.228316, 0.222101]
        self.vacuum_pick_plastic = [-0.094402, -0.06273, 1.32702, -0.671678, -0.66896, 0.228123, 0.222038]
        self.vacuum_pick_plastic_down = [-0.094402, -0.06273, 1.325, -0.671678, -0.66896, 0.228123, 0.222038]
        self.vacuum_raise_plastic = [-0.094379, -0.06277, 1.338345, -0.671616, -0.669021, 0.228141, 0.222025]
        self.vacuum_slide_plastic_to_left_1 = [-0.094356, -0.150395, 1.337414, -0.67282, -0.669026, 0.226228, 0.220313]
        self.vacuum_slide_plastic_to_left_2 = [-0.094458, -0.169041, 1.31232, -0.673183, -0.66889, 0.225682, 0.220178]
        self.vacuum_slide_plastic_to_left_3 = [-0.094584, -0.360521, 1.311543, -0.673824, -0.669155, 0.222796, 0.220349]
        self.vacuum_slide_plastic_leave = [-0.094688, -0.360438, 1.279992, -0.673991, -0.668938, 0.22274, 0.220559]
        self.vacuum_slide_plastic_return_1 = [-0.094605, -0.360505, 1.30538, -0.673866, -0.669086, 0.222826, 0.220402]
        self.vacuum_slide_plastic_return_2 = [-0.094385, -0.192304, 1.353041, -0.673076, -0.669211, 0.225283, 0.219938]

        self.on_strength_plate_up = [-0.036165, -0.393936, 1.347002, -0.711735, -0.660946, 0.163322, 0.172944]
        self.on_strength_plate_down = [-0.036391, -0.393818, 1.274475, -0.712154, -0.660444, 0.163194, 0.17326]
        self.return_to_imaging = [0.346632, -0.186022, 1.30875, -0.382517, 0.923937, -0.001841, 0.004168]
        self.return_to_imaging_2 = [0.490965, -0.120166, 1.26818, -0.528279, 0.849059, 0.000983, 0.004446]

        self.view_pouch_on_strength = [-0.013873, -0.255513, 1.286548, -0.667315, -0.687035, 0.205625, 0.200975]
         
        self.to_dispose_1 = [0.241486, -0.327828, 1.287803, 0.050937, -0.993542, -0.096524, 0.03103]
        self.to_dispose_2 = [-0.083833, -0.427706, 1.233464, -0.288517, -0.856398, -0.176733, 0.390007]
        self.on_dispose = [-0.162181, -0.37927, 1.209768, -0.423817, -0.738388, -0.174513, 0.494678]
        self.on_dispose_leave_middle = [-0.157333, -0.448179, 1.210342, -0.510421, -0.766033, -0.192452, 0.340037]
        self.on_dispose_leave = [-0.165765, -0.553165, 1.109779, -0.655006, -0.717474, -0.105037, 0.21252]

        self.d6_close_up = [0.998117, 0.000856, 0.417528, 0.004308, 0.991599, 0.00167, 0.12927]
        self.d6_close_down = [0.998076, 0.001207, 0.378152, 0.004193, 0.991597, 0.00199, 0.129283]
        self.d6_close_half = [0.559361, 0.000834, 0.37697, -0.004167, -0.969404, -0.00036, 0.245436]
        self.d6_close_closed = [0.350219, 0.001958, 0.373114, -0.003815, -0.853954, -0.001, 0.520333]

        self.to_drop_1 = [0.533799, -0.173287, 1.203803, -0.464458, 0.885584, 0.001427, 0.004152]
        self.to_drop_2 = [0.346546, -0.291555, 1.256498, 0.009896, 0.999941, 0.000391, 0.004533]
        self.on_drop = [0.152082, -0.292145, 1.256507, 0.009933, 0.99994, -0.002392, 0.003842]
        
        self.to_instruments_1 = [0.500967, -0.208749, 1.103049, -0.379738, 0.925087, 0.001132, 0.003604]
        self.above_scale = [0.084298, -0.27661, 1.236543, -0.690691, -0.686761, 0.161216, 0.159107]
        self.above_scale_half = [0.164275, -0.275954, 1.121205, -0.673176, -0.66755, 0.223953, 0.225955]
        self.on_scale = [0.085634, -0.27525, 1.095338, -0.650485, -0.643828, 0.283537, 0.286288]

        # NEW STRENGHT POSITIONS
        self.eg_to_strength_1 = [0.114897, -0.296926, 1.327567, -0.002459, -0.869463, 0.02644, 0.493284]
        self.eg_to_strength_2 = [-0.163284, -0.734736, 1.248664, -0.298016, -0.762066, 0.206283, 0.536553]
        self.eg_to_strength_3 = [-0.257824, -0.592709, 1.281942, -0.059625, -0.82521, 0.218417, 0.517462]
        self.eg_to_strength_4 = [-0.240219, -0.303836, 1.299381, -0.196905, -0.777554, 0.074733, 0.592498]
        self.eg_to_strength_5 = [-0.178067, -0.34708, 1.248293, -0.420975, -0.69268, 0.398838, 0.428839]
        self.eg_to_strength_6 = [-0.175343, -0.335516, 1.210022, -0.548606, -0.643791, 0.400794, 0.352035]
        self.eg_away_strength_1 = [-0.172866, -0.333685, 1.2951, -0.524825, -0.553757, 0.525475, 0.376546]
        self.eg_away_strength_2 = [0.012887, -0.334656, 1.313691, -0.451879, -0.702492, 0.460892, 0.299817]
        self.eg_away_strength_3 = [0.263397, -0.315301, 1.372683, 0.027426, -0.997539, -0.031649, 0.056225]

        self.maxAgeForBoxesRecognitionSecs = 2

        self.xmin = 0.45
        self.xmax = 0.85
        self.ymin = -0.26
        self.ymax = 0.26
        self.xmid = (self.xmin + self.xmax)/2
        self.ymid = (self.ymin + self.ymax)/2
        self.zref = 0.5
        self.drawer6plateZ = 0.35
        self.d6_above = [0.78646, 0.004805, 0.615243, 0.704002, -0.710198, -0.000841, 0.000369]

        self.RPY_XLimit = 0.57
        self.RPY_BL = [-136,0,20]
        self.RPY_BR = [-141,0,90]
        self.RPY_UL = [-145,0,0]
        self.RPY_UR = [-166,0,0]

        rospy.loginfo('SUD Posta Controller V2 INIT Completed ----------------------------------')

    # @@@ Internal Methods

    def get_current_timestamp_message(self):
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        return 'CurrentTime: {0}'.format(current_time)

    def input_commands_callback(self, data):
        
        self.latest_command_string = str(data.data)

        valid_command, response, command, action_name, agent, startstop, p1, p2 = self.parse_command(self.latest_command_string)

        if(valid_command):

            action = Action(agent, command, action_name, startstop, p1, p2)

            if(command==PostaCommands.StopCurrentRobotTrajectory):
                if(self.moveit_trajectory_execution_status=='MONITOR'):
                    self.stop_robot_motion_flag = True
                    rospy.loginfo('STOP Robot Command RECEIVED --> sent to MoveIT')
                else:
                    rospy.logwarn('STOP robot command received, but MOVEIT status is set to [{0}], nothing to stop'.format(self.moveit_trajectory_execution_status))

            elif(command==PostaCommands.StopAll):
                self.stop_robot_motion_flag = True
                self.robot_action_list = []
                self.mark10_action_list = []
                self.scale_action_list = []
                self.send_error_msg('ALL SYSTEM WAS STOPPED', 'INFO')

            elif startstop == 'queue':
                self.latest_command = command
                self.latest_command_parameter_1 = p1
                self.latest_command_parameter_2 = p2
                
                action.status = 'ready'
                if action.agent == 'robot':
                    self.robot_action_list.append(action)

                elif action.agent == 'scale':
                    self.scale_action_list.append(action)

                elif action.agent == 'mark10':
                    self.mark10_action_list.append(action)

            elif startstop == 'start':
                #Stop current ongoing action and start the new one
                if action.agent == 'robot':
                    self.stop_robot_motion_flag = True
                    rospy.sleep(1)
                    self.stop_robot_motion_flag = False
                    self.robot_action_list.insert(0, action)

                elif action.agent == 'scale':
                    # Nothing to stop for now
                    self.scale_action_list.insert(0, action)

                elif action.agent == 'mark10':
                    # Nothing to stop for now
                    self.mark10_action_list.insert(0, action)
                  
            elif startstop == 'stop':
                    if agent == 'robot':
                        # If that action is ongoing, stop the action and delete everithing in the queue
                        if (self.robot_action_list[0].command == command and 
                            self.robot_action_list[0].status == 'ongoing'):

                            self.send_error_msg('ROBOT EXECUTION STOPPED', 'INFO')
                            self.stop_robot_motion_flag = True
                            self.robot_action_list = []
                            rospy.sleep(0.2)
                            self.agents_status['robot'] = (action_name, 'stopped', p1, p2)
                            
                        else:
                            err_str = 'Cannot stop {} action since is not in execution'.format(action_name)
                            self.send_error_msg(err_str, 'ERROR')
                    elif agent == 'scale':
                        # TO DO... actually nothing to stop for now
                        self.scale_action_list = []
                        pass
                    elif agent == 'mark10':
                        # TO DO... actually nothing to stop for now
                        self.mark10_action_list = []
                        pass
                        
            else:
                err_str = 'Recieved Action with status {0}. Only "start", "queue" or "stop" status are admissible when giving an action.'.format(action.status)
                self.send_error_msg(err_str, 'ERROR')

    def robotiq_status_listener_callback(self,status):
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
        else:
            self.gripper_in_position = False

        self.gripper_position_value = status.gPR
        self.gripper_position_value_mm = 86.6 - 0.38*self.gripper_position_value

    def pouches_bbox_callback(self, data):
        self.bounding_boxes_header = data.header
        self.bounding_boxes = []

        for box_org in data.bounding_boxes:
            self.bounding_boxes.append(box_org)

        # This method checks if there are valid pouches in view
        self.IsPouchDetected()
    
    def darknet_bbox_callback(self, data):
        self.darknet_bounding_boxes = data.bounding_boxes
        self.darknet_header = data.header

    def MTweight_callback(self, data):
        self.ScaleWeight = data.illuminance
        self.ScaleWeightROSTimeStamp = data.header.stamp
        self.ScaleWeightTimestamp = time.time()
        
    def MoveitExecuteTrajectoryFeedbackCallback(self,data):
        self.moveit_trajectory_execution_status = data.feedback.state
        #rospy.loginfo('MOVEIT Trajectory Execution Status: {0}'.format(self.moveit_trajectory_execution_status)) 

    def generate_status_json(self):

        status = OrderedDict(
            timestamp=time.time(),
            commands_in_queue=len(self.commands_queue),
            gripper_active=self.gripper_active,
            gripper_in_position=self.gripper_in_position,
            gripper_aperture= self.gripper_position_value,
            gripper_aperture_mm= self.gripper_position_value_mm
        )

        status['robot'] = self.robot_motion_status
        status['moveit_trajectory_status'] = self.moveit_trajectory_execution_status

        for i in range(0,6):
            dNum = i+1
            dNumString = 'drawer_{0}'.format(dNum)
            if(self.current_drawer_number==dNum):
                if(self.current_drawer_isopen):
                    status[dNumString] = 'open'
                else:
                    status[dNumString] = 'closed'
            else:
                status[dNumString] = 'closed'
                
        status["number_of_detected_pouches"]  = self.number_of_pouches_detected
        status['pouch_in_gripper'] = self.pouch_in_gripper

        status['scale_weight'] = self.ScaleWeight
        status['scale_timestamp'] = self.ScaleWeightTimestamp

        status['posta_client_online'] = self.posta_client_online
        status['mark10_instrument_read'] = self.mark10_instrument_ready
        status['mark10_test_completed'] = self.mark10_test_completed
        status['mark10_plate_distance'] = self.mark10_plate_distance
        status['mark10_plate_moving_up'] = self.mark10_plate_moving_up
        
        # Error Status
        status['error_msg'] = self.error_msg
        status['moveit_error_msg'] = self.moveit_error_msg

        # Agents Status
        status['agents_status'] = json.dumps(self.agents_status)

        # Action List Status
        status['robot_action_list'] = json.dumps(self.send_action_list('robot'))
        status['mark10_action_list'] = json.dumps(self.send_action_list('mark10'))
        status['scale_action_list'] = json.dumps(self.send_action_list('scale'))

        status_json = json.dumps(status)

        # Save to local file
        with open(self.local_status_file, 'w') as file:
            file.write(status_json)

        return status_json

    def send_action_list(self, agent):
        l = []
        if agent == 'robot':
            for action in self.robot_action_list:
                l.append((action.name, action.status))
        elif agent == 'mark10':
            for action in self.mark10_action_list:
                l.append((action.name, action.status))
        elif agent == 'scale':
            for action in self.scale_action_list:
                l.append((action.name, action.status))
        return l

    def read_last_status_from_file(self):
        rospy.loginfo('Reading last saved status from file {0}'.format(self.local_status_file))
        status_json_read = ''

        try:
            with open(self.local_status_file, 'r') as file:
                status_json_read = file.read()

            rospy.loginfo('Local status file found')

            if(status_json_read is not None):
                if(len(status_json_read)>0):

                    status = json.loads(status_json_read)

                    # Read status of drawers
                    for i in range(6):
                        drawer_name = 'drawer_{0}'.format(i+1)
                        if(drawer_name in status):
                            if(status[drawer_name]=='open'):
                                rospy.loginfo('Drawer {0} is OPEN'.format(i+1))
                                self.current_drawer_number = i+1
                                self.current_drawer_isopen = True

                    # Read pouch status
                    self.pouch_in_gripper = status['pouch_in_gripper']


        except:
            rospy.logwarn('No local status file found, starting from scratch')

    def parse_command(self,command_string):

        if(len(command_string)>0):
            
            values = command_string.split('|')

            # Checking also if len if all necessary parameters are given
            if(values[0] in PostaCommands._member_names_ and len(values)>=3):
                
                name = values[0]
                command = PostaCommands[values[0]]
                agent = values[1]
                startstop = values[2]
                parameter1 = None
                parameter2 = None

                if(len(values)>3):
                    parameter1 = values[3]
                    
                if(len(values)>4):
                    parameter2 = values[4]

                return True,'',command, name, agent, startstop, parameter1, parameter2
                
            else:
                return False,'Received command not VALID: {0}'.format(self.latest_command_string),None,None,None
        else:
            return False,'Received command has zero length',None,None,None
    
    def tray_to_world_coordinates(self, x, y):
        pC = [
            (self.pM0UL[0] - y)/1000.0,
            (self.pM0UL[1] - x)/1000.0
        ]
        return pC
    
    def generate_pickup_pose(self, x, y, z):
        
        # use the BR (r x below the limit)
        rUDeg = self.RPY_BR

        # if X is above the limit use interpolation between UR and UL
        if(x>=self.RPY_XLimit):
            # use the UR/UR interpolation
            rUDeg = interpolate_in_array(self.RPY_UR,self.RPY_UL,self.ymin,self.ymax,y)     
        
        rF = [DegToRad(rUDeg[0]),DegToRad(rUDeg[1]),DegToRad(rUDeg[2])]

        # Transform to quaternion
        qF = quaternion_from_euler(rF[0],rF[1],rF[2])

        # Generate the output pose
        pOut = [x,y,z,qF[0],qF[1],qF[2],qF[3]]
        
        return pOut

    def zero_scale(self):
        msg = String()
        msg.data = "zero_scale"
        self.WeightCommandPublisher.publish(msg)
        return True
    
    def send_error_msg(self, msg, type, log=True):
        # Each Error Message will be
        # 'TYPE|MESSAGE|TIMESTAMP'

        msg = msg.replace(',', '').replace(':', '')

        if (type == 'ERROR' or type == 'WARNING' or type == 'INFO'):
            
            self.error_msg = '{0}|{1}|{2}'.format(type, msg, time.time())
        elif type == 'MOVEIT ERROR':
            self.moveit_error_msg = '{0}|{1}|{2}'.format(type, msg, time.time())
        else:
            raise ValueError('error type is wrong!')
        if log:
            rospy.logerr(msg)
    

    # @@@ Low Level Motion Methods

    def compute_deltaz_for_drawer(self, drawer_number):
        # Compute the delta_z to apply to every pose for the specified drawer
        delta_z = self.distance_between_drawers * (6 - drawer_number)
        return delta_z
    
    def execute_cartesian_path_and_check(self, waypoints, speed_scaling, max_retrials = 3):    
        mg = self.move_group
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)
        cnt = 0
        while cnt<max_retrials:
            cnt += 1
            self.robot_motion_status = 'moving'
            response = mg.execute(plan, wait=True)
            if(response):
                self.robot_motion_status = 'idle'
                return True
            rospy.sleep(0.5)
        
        err_str = 'FAILED to execute_cartesian_path after {} retrials'.format(max_retrials)
        self.send_error_msg(err_str, 'MOVEIT ERROR')
        self.robot_motion_status = 'error'
        return False
    
    def execute_cartesian_path_and_check_async(self, waypoints, speed_scaling, max_retrials = 3):    
        mg = self.move_group
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)
        cnt = 0
        while cnt<max_retrials:
            cnt += 1

            self.robot_motion_status = 'moving'
            
            # Execute the trajectory in a non-blocking manner
            moveit_original_status = self.moveit_trajectory_execution_status # store the original status
            #rospy.loginfo('MOVEIT Starting Plan Execution ASYNC')
            mg.execute(plan, wait=False)

            max_wait_secs = 10
            status_changed_to_monitor = False
            max_time_elapsed = False
            start_time = time.time()
            
            # Wait for the status to change
            while((not status_changed_to_monitor) and (not max_time_elapsed)):
                rospy.sleep(0.1)
                status_changed_to_monitor = self.moveit_trajectory_execution_status=='MONITOR'
                max_time_elapsed = (time.time()-start_time)>max_wait_secs

            if(status_changed_to_monitor):
                # now wait for the execution to complete
                #rospy.loginfo('MOVEIT Status change OK --> from {0} to {1}'.format(moveit_original_status, self.moveit_trajectory_execution_status))
                max_wait_secs = 20
                start_time = time.time()
                status_changed_to_idle = False
                max_time_elapsed = False
                while((not status_changed_to_idle) and (not max_time_elapsed)):
                    
                    status_changed_to_idle = self.moveit_trajectory_execution_status == 'IDLE'

                    max_time_elapsed = (time.time()-start_time)>max_wait_secs
                    rospy.sleep(0.2)

                    # check if an interrupt has been received
                    if(self.stop_robot_motion_flag):
                        self.stop_robot_motion_flag = False
                        err_str = 'STOP INTERRUPT received in the low level motion function'
                        self.send_error_msg(err_str, 'MOVEIT ERROR')
                        mg.stop()
                        self.robot_motion_status = 'stopped'
                        return False

                if(status_changed_to_idle):
                    #rospy.loginfo('MOVEIT Trajectory Execution Completed Successfully, status changed back to IDLE')
                    self.robot_motion_status = 'idle'
                    return True
                
                elif(max_time_elapsed):
                    self.robot_motion_status = 'idle'
                    err_str = 'Status did not switch back to IDLE after starting trajectory for {0} secs'.format(max_wait_secs)
                    self.send_error_msg(err_str, 'MOVEIT ERROR')

                    return False

            elif(max_time_elapsed):
                err_str = 'After calling mg.execute the status did not change to MONITOR after {0} secs'.format(max_wait_secs)
                self.send_error_msg(err_str, 'MOVEIT ERROR')

                self.robot_motion_status = 'idle'
                return False

    def go_to_position_and_check(self, pose_array, speed_scaling = 0.05, max_retrials = 3):
        mg = self.move_group        
        waypoints = []
        waypoints.append(ArrayToPose(pose_array,0))
        return self.execute_cartesian_path_and_check(waypoints, speed_scaling, max_retrials)
        
    def set_digital_output(self, channel, state):
        self.set_io_client(1,channel,state)


    # @@@ Gripper Methods

    def reset_gripper(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 0
        self.robotiq_pub.publish(self.command)
        return

    def activate_gripper(self):        
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.robotiq_pub.publish(self.command)
        return

    def activate_gripper_and_check(self):
        
        max_count = 5
        cnt = 0
        #self.gripper_active = False # force gripper_active = false

        if(self.gripper_active):
            rospy.loginfo('Gripper is already active, no need to initialize')
        else:
            rospy.loginfo('Gripper is NOT active, proceed with initialization')
        
        # When initializing the gripper will open, so it will drop anything it previously had
        self.pouch_in_gripper = False

        while not self.gripper_active:
            cnt += 1
            rospy.loginfo('   ITER[{}] RESET Gripper...'.format(cnt))
            self.reset_gripper()
            rospy.sleep(1)
            rospy.loginfo('   ITER[{}] ACTIVATE Gripper...'.format(cnt))
            self.activate_gripper()
            rospy.sleep(3)
            if(cnt>max_count):
                rospy.logerr('   Gripper activation max iterasions exceeded!!!')
                break
            if(self.gripper_active):
                rospy.loginfo('   Gripper active, put at at 20mm')
                self.set_gripper_aperture(20)

        return self.gripper_active

    def open_gripper(self):
        self.command.rPR = 0
        self.pouch_in_gripper = False
        self.robotiq_pub.publish(self.command)
        rospy.sleep(0.5)
        return

    def close_gripper(self):
        self.command.rPR = 255
        self.robotiq_pub.publish(self.command)
        rospy.sleep(0.5)
        return

    def set_gripper_aperture(self, aperture_mm):
        x_command = (86.6-aperture_mm)/0.38
        if(x_command<0):
            x_command = 0
        if(x_command>255):
            x_command = 255
        self.set_gripper_position(x_command)

    def set_gripper_position(self, position):
        # p030 = 76mm
        # p070 = 62mm
        # p100 = 50mm
        # p120 = 40mm
        # p140 = 34mm
        # p170 = 20mm
        # p200 = 8mm
        # p230 = 2
        self.command.rPR = position
        self.robotiq_pub.publish(self.command)
        rospy.sleep(0.8)
        return


    # @@@ Vision Methods

    def IsPouchDetected(self):
        # wait for detected pouches
        nBBoxes = 0
        detection_age_sec = 0

        if(self.bounding_boxes_header is not None):
            detection_age_sec = (rospy.get_rostime() - self.bounding_boxes_header.stamp).to_sec()

        if self.bounding_boxes is not None:
            nBBoxes = len(self.bounding_boxes)

        if (nBBoxes>0) and (detection_age_sec>0) and (detection_age_sec<(self.maxAgeForBoxesRecognitionSecs)):
            self.number_of_pouches_detected = nBBoxes
        else:
            self.number_of_pouches_detected = 0

    def GetNextPouchToPickup(self):
        # just get the first pouch (for now). Structure of BoundingBox is below:
        #pBox = BoundingBox()
        #pBox.Class = box.Class
        #pBox.probability = box.probability
        #pBox.xmin  
        #pBox.ymin 
        #pBox.xmax 
        #pBox.ymax 
        if(self.bounding_boxes is not None and len(self.bounding_boxes) > 0 ):
            return self.bounding_boxes[0]
        else:
            self.error_msg = 'ERROR|Drawer is emtpy. No pouches are detected!'
            return None


    # @@@ Mark10 Methods

    def IsPostaServerOnline(self):
        if self.PostaClient.PingServer():
            self.posta_client_online = True
        else:
            self.posta_client_online = False
        return self.posta_client_online


    # @@@ High Level Motion Methods

    def open_drawer(self, drawer_number,speed_scaling = 0.05, max_retrials = 3 ):
        delta_z = self.compute_deltaz_for_drawer(drawer_number)
        mg = self.move_group
        
        # 01) go to the approach position and stop
        waypoints = []
        waypoints.append(ArrayToPose(self.drawer_raised_position))
        waypoints.append(ArrayToPose(self.drawer6_handle_approach,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # Set gripper aperture
        self.set_gripper_aperture(40)

        #02) engage handle and stop
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_handle_engaged,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.05, max_retrials):
            return False

        #03) open drawer until half way
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.03, max_retrials):
            return False

        #04) switch to re-engaged position
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open_up1,delta_z))
        waypoints.append(ArrayToPose(self.drawer6_half_open_up2,delta_z))
        waypoints.append(ArrayToPose(self.drawer6_half_open_reengaged,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        #05) open drawer fully
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_full_open,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.03, max_retrials):
            return False
        
        # let it equilibrate
        rospy.sleep(1)

        #06) return to center
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_full_open_up,delta_z))
        waypoints.append(ArrayToPose(self.drawer_raised_position))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        self.current_drawer_isopen = True

        return True

    def close_drawer(self, drawer_number,speed_scaling = 0.05, max_retrials = 3 ):
        delta_z = self.compute_deltaz_for_drawer(drawer_number)
        mg = self.move_group

        # 01) go to the approach position and stop
        waypoints = []
        waypoints.append(ArrayToPose(self.d6_close_up,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # 02) close the gripper
        self.set_gripper_aperture(0)

        # 03) close the drawer
        waypoints = []
        waypoints.append(ArrayToPose(self.d6_close_down,delta_z))
        waypoints.append(ArrayToPose(self.d6_close_half,delta_z))
        waypoints.append(ArrayToPose(self.d6_close_closed,delta_z))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.05, max_retrials):
            return False

        rospy.sleep(1)

        waypoints = []
        waypoints.append(ArrayToPose(self.d6_close_half,delta_z))
        waypoints.append(ArrayToPose(self.drawer_half_cabinet,0))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False
        
        self.current_drawer_isopen = False

        return True

    def go_to_drawer_imaging(self, drawer_number, speed_scaling = 0.05, max_retrials = 3):
        mg = self.move_group        
        delta_z = self.compute_deltaz_for_drawer(drawer_number)
        waypoints = []
        waypoints.append(ArrayToPose(self.drawer6_shoot_image,delta_z))
        #return self.execute_cartesian_path_and_check(waypoints, speed_scaling, max_retrials)
        return self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials)

    def GoAndPickupNextPouch(self):
        
        box = self.GetNextPouchToPickup()
        
        if(box is not None):
            pCenterMM = [
                (box.xmin + box.xmax)/2,
                (box.ymin + box.ymax)/2
            ]
            pPickupWorld = self.tray_to_world_coordinates(pCenterMM[0],pCenterMM[1])
            delta_z = self.compute_deltaz_for_drawer(self.current_drawer_number)
            pose_pickup_high = self.generate_pickup_pose(pPickupWorld[0], pPickupWorld[1], self.drawer6plateZ + delta_z + 0.07)
            pose_pickup = self.generate_pickup_pose(pPickupWorld[0], pPickupWorld[1], self.drawer6plateZ + delta_z +0.005)

            self.set_gripper_aperture(45)

            # Go the pickup pose through the intermediate waypoint
            waypoints = []
            waypoints.append(ArrayToPose(self.d6_above, delta_z))
            waypoints.append(ArrayToPose(pose_pickup_high))
            if not self.execute_cartesian_path_and_check_async(waypoints, 0.5,3):
                return False

            # Pick the pouch
            if not self.go_to_position_and_check(pose_pickup):
                return False
            
            self.set_gripper_aperture(self.gripper_aperture_forPouch)
            self.pouch_in_gripper = True

            # Go to intermediate position
            waypoints = []
            waypoints.append(ArrayToPose(pose_pickup_high))
            waypoints.append(ArrayToPose(self.d6_above, delta_z))
            if not self.execute_cartesian_path_and_check_async(waypoints, 0.5,3):
                return False
            
            return True

        else:
            err_str = 'Could not retrieve a valid bbox for pouch pickup'
            rospy.logerr(err_str)
            self.error_msg = 'ERROR|'+err_str
            return False
    
    def put_pouch_on_scale(self, speed_scaling=0.05, max_retrials = 3):
        
        delta_z = self.compute_deltaz_for_drawer(self.current_drawer_number)

        waypoints = []
        waypoints.append(ArrayToPose(self.d6_above, delta_z))
        waypoints.append(ArrayToPose(self.to_instruments_1))
        waypoints.append(ArrayToPose(self.above_scale_half))
        waypoints.append(ArrayToPose(self.on_scale))

        a = self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials)
        
        if not a:
            return False

        # open gripper
        self.set_gripper_aperture(60)
        self.pouch_in_gripper = False

        # Going away from the scale
        waypoints = [ArrayToPose(self.above_scale)]
        if not self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials):
            return False

        return True

    def pick_pouch_from_scale(self, speed_scaling=0.05, max_retrials = 3):
        
        waypoints = []
        waypoints.append(ArrayToPose(self.above_scale))
        waypoints.append(ArrayToPose(self.on_scale))
        if not self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials):
            return False

        # close gripper
        self.set_gripper_aperture(self.gripper_aperture_forPouch)
        self.pouch_in_gripper = True

        waypoints = []
        waypoints.append(ArrayToPose(self.above_scale))
        if not self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials):
            return False

        return True

    def dispose_paper(self, speed_scaling = 0.05, max_retrials = 3):
        mg = self.move_group        
        
        # approach the dispose point - reworked on Jan23 with new plate
        waypoints = []
        waypoints.append(ArrayToPose(self.to_dispose_1,0))
        waypoints.append(ArrayToPose(self.to_dispose_2,0))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # Open the gripper
        self.set_gripper_aperture(60)

        # go to the dispose point
        waypoints = []
        waypoints.append(ArrayToPose(self.on_dispose,0))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.05, max_retrials):
            return False

        # Close the gripper
        self.set_gripper_aperture(0)

        # lift the papers to dispose
        waypoints = []
        waypoints.append(ArrayToPose(self.on_dispose_leave_middle))
        waypoints.append(ArrayToPose(self.on_dispose_leave))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # Open the gripper
        self.set_gripper_aperture(60)
        #rospy.sleep(0.1) # wait for the paper to fall off

        # Return to imaging
        waypoints = []
        waypoints.append(ArrayToPose(self.to_dispose_2))
        waypoints.append(ArrayToPose(self.to_dispose_1,0))
        waypoints.append(ArrayToPose(self.drawer6_shoot_image,0))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        return True

    def put_pouch_on_strength(self, speed_scaling = 0.05,  max_retrials = 3):
        mg = self.move_group        
        waypoints = []
        
        # Move the pouch onto the strength plate
        waypoints = []
        waypoints.append(ArrayToPose(self.to_strength_4))
        waypoints.append(ArrayToPose(self.on_strength,0))

        if not self.execute_cartesian_path_and_check_async(waypoints, 0.05, max_retrials):
            return False
        
        # Open Gripper
        self.set_gripper_aperture(45)
        self.pouch_in_gripper = False

        return True

    def go_from_strength_to_imaging(self, speed_scaling=0.05, max_retrials = 3):
        mg = self.move_group

        # Computing deltaz to go to the right drawer imaging
        delta_z = self.compute_deltaz_for_drawer(self.current_drawer_number)

        waypoints = []        
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.to_strength_4,0))
        waypoints.append(ArrayToPose(self.to_strength_3,0))
        waypoints.append(ArrayToPose(self.to_strength_2,0))
        waypoints.append(ArrayToPose(self.to_strength_1,0))
        waypoints.append(ArrayToPose(self.drawer6_shoot_image,delta_z))
        return self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials)
    
    def go_to_rest_pose(self, speed_scaling=0.3, max_retrials = 3):
        # Computing deltaz to go to the right drawer imaging
        delta_z = self.compute_deltaz_for_drawer(self.current_drawer_number)
        waypoints = []  
        waypoints.append(ArrayToPose(self.drawer6_shoot_image,delta_z))
        return self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials)

    def put_paper_sheet(self, current_robot_position, with_pouch=0, speed_scaling=0.05, max_retrials = 3):
        mg = self.move_group        
        
        # depending on where the robot is, bring him closer
        waypoints = []
        if(current_robot_position=='on_strength'):
            waypoints.append(ArrayToPose(self.to_strength_4))
            waypoints.append(ArrayToPose(self.above_scale))
        waypoints.append(ArrayToPose(self.vacuum_goingto_2))
        waypoints.append(ArrayToPose(self.vacuum_above))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # Open the gripper
        self.set_gripper_aperture(60)

        # Go to the pickup position
        waypoints = []
        #waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.vacuum_pick))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.05, max_retrials):
            return False
        
        # Close the gripper and turn on vacuum
        self.set_gripper_aperture(0)
        rospy.sleep(0.1)

        # Put the paper on the tray
        waypoints = []
        waypoints.append(ArrayToPose(self.vacuum_pick_down))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False
        self.set_digital_output(0,1)
        rospy.sleep(0.5)

        waypoints = []
        waypoints.append(ArrayToPose(self.vacuum_above))
        if not self.execute_cartesian_path_and_check_async(waypoints,0.005, max_retrials):
            return False
        
        waypoints = []
        waypoints.append(ArrayToPose(self.vacuum_abovePlate))
        waypoints.append(ArrayToPose(self.vacuum_abovePlate_pushOnColumn))
        
        if(with_pouch==0):
            waypoints.append(ArrayToPose(self.vacuum_onPlate))
        else:
            waypoints.append(ArrayToPose(self.vacuum_onPlate_withPouch))

        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        # Turn off vacuum
        self.set_digital_output(0,0)
        
        # Put back the vacuum holder
        waypoints = []
        #waypoints.append(ArrayToPose(self.on_strength_plate_up,0))
        waypoints.append(ArrayToPose(self.vacuum_abovePlate))
        waypoints.append(ArrayToPose(self.vacuum_above))
        waypoints.append(ArrayToPose(self.vacuum_pick))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False
        
        # Open the gripper with the vacuum on to stabilize it
        self.set_digital_output(0,1)
        rospy.sleep(0.1)
        self.set_gripper_aperture(60)
        rospy.sleep(0.1)
        self.set_digital_output(0,0)

        # Move back to 
        waypoints = []
        waypoints.append(ArrayToPose(self.vacuum_above,0))
        waypoints.append(ArrayToPose(self.vacuum_goingto_2,0))
        if not self.execute_cartesian_path_and_check_async(waypoints,speed_scaling, max_retrials):
            return False

        return True

    def put_pouch_on_strength_V2(self, speed_scaling = 0.05,  max_retrials = 3):
        mg = self.move_group        
    
        # Move the pouch onto the strength plate
        waypoints = []
        waypoints.append(ArrayToPose(self.above_scale))
        waypoints.append(ArrayToPose(self.eg_to_strength_1))
        waypoints.append(ArrayToPose(self.eg_to_strength_2))
        waypoints.append(ArrayToPose(self.eg_to_strength_3))
        waypoints.append(ArrayToPose(self.eg_to_strength_4))
        waypoints.append(ArrayToPose(self.eg_to_strength_5))
        waypoints.append(ArrayToPose(self.eg_to_strength_6))

        if not self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials):
            return False
        
        # Open Gripper
        self.set_gripper_aperture(45)
        self.pouch_in_gripper = False

         # Move away from the strength plate
        waypoints = []
        waypoints.append(ArrayToPose(self.eg_away_strength_1))
        waypoints.append(ArrayToPose(self.eg_away_strength_2))
        waypoints.append(ArrayToPose(self.eg_away_strength_3))

        if not self.execute_cartesian_path_and_check_async(waypoints, speed_scaling, max_retrials):
            return False

        return True
    

    # @@@ Main Methods

    def StatusPublishingThread(self):
        while not rospy.is_shutdown():
            # publish the latest status
            self.sud_posta_state_publisher.publish(self.generate_status_json())

            rospy.sleep(0.5)

    def PostaClientUpdateThread(self):
        while not rospy.is_shutdown():
            # Continously check for status updates 
            self.posta_client_online = self.PostaClient.PingServer()
            rospy.sleep(0.1)

            if(self.posta_client_online):
                self.mark10_instrument_ready = self.PostaClient.IsInstrumentReady()
                rospy.sleep(0.1)
                self.mark10_test_completed = self.PostaClient.IsTestComplete()
                rospy.sleep(0.1)
                self.mark10_plate_distance = self.PostaClient.GetPlateDistance()
                rospy.sleep(0.1)
                self.mark10_plate_moving_up = self.PostaClient.IsPlateMovingUp()
                rospy.sleep(0.1)
        
            rospy.sleep(0.1)

    def ControllerStart(self):
        rospy.loginfo('SUD Posta Controller V2 START')

        # read last status from file
        self.read_last_status_from_file()
        
        # Actions Lists
        self.robot_action_list = []
        self.mark10_action_list = []
        self.scale_action_list = []
        self.all_action_list = {'robot': self.robot_action_list, 
                                'mark10': self.mark10_action_list,
                                'scale': self.scale_action_list}

        # Starting Threads
        self.status_thread = threading.Thread(target=self.StatusPublishingThread)
        self.status_thread.start()

        self.posta_client_update_thread = threading.Thread(target=self.PostaClientUpdateThread)
        self.posta_client_update_thread.start()

        self.robot_thread = threading.Thread(target=self.RobotThread)
        self.robot_thread.start()

        self.scale_thread = threading.Thread(target=self.ScaleThread)
        self.scale_thread.start()

        self.mark10_thread = threading.Thread(target=self.Mark10Thread)
        self.mark10_thread.start()

        # If a new instrument has to be added, just create a new thread
        # using as target the relative function (RobotThread if new instrument
        # is a robot, ...)

    def RobotThread(self):

        rospy.loginfo('ROBOT THREAD STARTED')
        while not rospy.is_shutdown():
            
            if len(self.robot_action_list) > 0:

                # Consider first action in queue
                action = self.robot_action_list[0]
        
                # Executing the action
                current_command = action.command
                action.status = 'ongoing'

                # Robot Ongoing Status
                self.agents_status['robot'] = (action.name, action.status, action.p1, action.p2)

                if(current_command==PostaCommands.OpenGripper):
                    self.open_gripper()
                    action.status = 'completed'
                    rospy.loginfo('--> OpenGripper command Executed')
                
                elif(current_command==PostaCommands.CloseGripper):
                    self.close_gripper()
                    action.status = 'completed'
                    rospy.loginfo('--> CloseGripper command Executed')
                
                elif(current_command==PostaCommands.SetGripperApertureMM):
                    v = int(action.p1)
                    rospy.loginfo('--> Gripper Aperture set to {0} mm'.format(action.p1))
                    
                    if v <= 86 and v >= 0:
                        self.set_gripper_aperture(v)
                    else:
                        self.send_error_msg('Gripper Aperture Range not admissible (range: 0-86)', 'ERROR')

                elif(current_command==PostaCommands.OpenDrawer):
                    target_drawer_number = int(action.p1)

                    if(self.current_drawer_isopen):
                        if(self.current_drawer_number==target_drawer_number):
                            rospy.loginfo("Drawer [{0}] is already open, no action".format(target_drawer_number))

                        else:
                            err_str = 'Drawer [{0}] cannot be opened while drawer [{1}] is open, close this one first'.format(target_drawer_number, self.current_drawer_number)
                            self.send_error_msg(err_str, 'ERROR')
                            action.status = 'error'
                    else:
                        self.current_drawer_number = target_drawer_number
                        rospy.loginfo('No currently opened drawer, proceed to open drawer {0}...'.format(self.current_drawer_number))                                
                        
                        action_check = self.open_drawer(self.current_drawer_number)
                        if action_check:
                            rospy.loginfo('--> Opening Drawer {0} COMPLETED'.format(self.current_drawer_number))
                            action.status = 'completed'
                        else:
                            err_str = 'Opening Drawer {0} NOT COMPLETED'.format(self.current_drawer_number)
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'
                        
                elif(current_command==PostaCommands.CloseDrawer):
                    target_drawer_number = int(action.p1)

                    if(self.current_drawer_isopen):
                        self.current_drawer_number = target_drawer_number
                        rospy.loginfo('Drawer {0} is currently open, closing it down...'.format(self.current_drawer_number))

                        action_check = self.close_drawer(self.current_drawer_number)
                        if action_check:
                            rospy.loginfo('--> Closing Drawer {0} COMPLETED'.format(self.current_drawer_number))
                            action.status = 'completed'
                        else:
                            err_str = 'Closing Drawer {0} NOT COMPLETED'.format(self.current_drawer_number)
                            self.send_error_msg(err_str, 'ERROR')

                            if not action.status == 'stopped':
                                action.status = 'error'
                    else:
                        # no open drawer, nothing to close
                        self.current_drawer_number = target_drawer_number
                        err_str = 'Drawer {0} is already closed, no action'.format(self.current_drawer_number)
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'                             

                elif(current_command==PostaCommands.GoToDrawerImaging):
                    target_drawer_number = int(action.p1)
                    if (self.current_drawer_number==target_drawer_number) and (self.current_drawer_isopen):        
                        rospy.loginfo('Going to Drawer {0} imaging position'.format(self.current_drawer_number))

                        # Execute the action
                        action_check = self.go_to_drawer_imaging(self.current_drawer_number)
                        if action_check:
                            rospy.loginfo('--> Going to drawer imaging COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Going to drawer {} imaging NOT COMPLETED'.format(self.current_drawer_number)
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'
                    else:
                        err_str = 'The target drawer {0} is not open. I am not going to that imaging position'.format(target_drawer_number)
                        self.send_error_msg(err_str, 'WARNING')
                        action.status = 'error'
                
                elif(current_command==PostaCommands.PickNextPouch):
                    if (self.current_drawer_number>0) and (self.current_drawer_isopen):
                        rospy.loginfo('Going to pickup next pouch')

                        # Execute the action
                        action_check = self.GoAndPickupNextPouch()
                        if action_check:
                            rospy.loginfo('--> Pouch pickup COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Pouch pickup was not completed'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'
                    else:
                        action.status = 'error'
                        err_str = 'Cannot pickup pouch, drawer is not open'
                        self.send_error_msg(err_str, 'ERROR')
                
                elif(current_command==PostaCommands.PutPouchOnScale):
                    if(self.pouch_in_gripper):
                        rospy.loginfo('Going to put pouch on scale')
                        
                        # Execute the action
                        action_check = self.put_pouch_on_scale()
                        if action_check:
                            rospy.loginfo('--> Pouch on scale COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Put puoch on scale NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'

                    else:
                        err_str = 'There is no pouch in the gripper right now, nothing to put on scale'
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'

                elif(current_command==PostaCommands.PickPouchFromScale):
                    if(not self.pouch_in_gripper):
                        rospy.loginfo('Going to pick pouch from scale')

                        # Execute the action
                        action_check = self.pick_pouch_from_scale()
                        if action_check:
                            rospy.loginfo('--> Pick pouch from scale COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Pick pouch from scale NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'
                        
                    else:
                        err_str = 'There is already a pouch in the gripper, cannot pickup a new one'
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'

                elif(current_command == PostaCommands.PutPouchOnStrengthPlate):
                    if(self.pouch_in_gripper):
                        rospy.loginfo('Going to put pouch on strength plate')

                        # Execute the action
                        action_check = self.put_pouch_on_strength_V2(speed_scaling = 0.20)
                        if action_check:
                            rospy.loginfo('--> Put pouch on strenght COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Put pouch on strenght NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'

                    else:
                        err_str = 'There is no pouch in gripper, nothing to put on strength plate'
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'

                elif(current_command == PostaCommands.PutPaperSheetOnStrength):
                    if(not self.pouch_in_gripper):
                        rospy.loginfo('Going to put put paper sheet on strength plate')

                        action_check = self.put_paper_sheet('on_strength')

                        if action_check:
                            rospy.loginfo('--> Put paper sheet on strenght COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Put paper sheet on strenght NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'

                    else:
                        err_str = 'There is currently a pouch in the gripper, cannot put a paper sheet'
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'

                elif(current_command == PostaCommands.DisposePaperSheetFromStrength):
                    if(not self.pouch_in_gripper):
                        rospy.loginfo('Going to dispose paper sheets')

                        action_check = self.dispose_paper()
                        if action_check:
                            rospy.loginfo('--> Dispose paper COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Dispose paper NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            if not action.status == 'stopped':
                                action.status = 'error'
                    else:
                        err_str = 'There is currently a pouch in the gripper, cannot dispose a paper sheet'
                        self.send_error_msg(err_str, 'ERROR')
                        action.status = 'error'

                elif(current_command == PostaCommands.GoFromStrengthToImaging):

                    action_check = self.go_from_strength_to_imaging()

                    if action_check:
                        rospy.loginfo('--> From Strenght to Imaging COMPLETED')
                        action.status = 'completed'
                    else:
                        err_str = 'From Strenght to Imaging NOT COMPLETED'
                        self.send_error_msg(err_str, 'ERROR')
                        if not action.status == 'stopped':
                            action.status = 'error'

                elif(current_command == PostaCommands.GoToRestPose):

                    action_check = self.go_to_rest_pose()
                    if action_check:
                        rospy.loginfo('--> Go to Rest Pose COMPLETED')
                        action.status = 'completed'
                    else:
                        err_str = 'Go to Rest Pose NOT COMPLETED'
                        self.send_error_msg(err_str, 'ERROR')
                        if not action.status == 'stopped':
                            action.status = 'error'

                else:
                    rospy.logwarn('COMMAND {0} NOT Yet Implemented'.format(current_command))
                
                # Robot Final Status
                self.agents_status['robot'] = (action.name, action.status, action.p1, action.p2)

                # Once action is done, remove it from the list
                if len(self.robot_action_list)>0:
                    self.robot_action_list.pop(0)

            rospy.sleep(0.5)

    def Mark10Thread(self):
        rospy.loginfo('MARK10 THREAD STARTED')
        while not rospy.is_shutdown():

            if len(self.mark10_action_list) > 0:
                action = self.mark10_action_list[0]
        
                # Executing the action
                current_command = action.command
                action.status = 'ongoing'

                # Mark10 Ongoing Status
                self.agents_status['mark10'] = (action.name, action.status, action.p1, action.p2)
                
                # Mark10 Final Status
                if(current_command == PostaCommands.StartStrengthTest):
                    if(self.mark10_instrument_ready and self.mark10_test_completed):
                        rospy.loginfo('Starting Mark10 Strength Test')
                        self.PostaClient.SetProductId('SUD_230101_001') # FAKE - find a way to get the right product id

                        action_check = self.PostaClient.StartStrength()

                        if action_check:
                            rospy.loginfo('--> Start Strengh Test COMPLETED')
                            action.status = 'completed'
                        else:
                            err_str = 'Start Strengh Test NOT COMPLETED'
                            self.send_error_msg(err_str, 'ERROR')
                            action.status = 'error'

                    else:
                        action.status = 'error'
                        err_str = 'Cannot start strength test, the instrument is not ready or test is not completed'
                        self.send_error_msg(err_str, 'ERROR')
            
                 # Mark10 Final Status
                
                self.agents_status['mark10'] = (action.name, action.status, action.p1, action.p2)

                if len(self.mark10_action_list)>0:
                    self.mark10_action_list.pop(0)

            rospy.sleep(0.5)

    def ScaleThread(self):
        rospy.loginfo('SCALE THREAD STARTED')
        while not rospy.is_shutdown():
            
            if len(self.scale_action_list) > 0:
                action = self.scale_action_list[0]
        
                # Executing the action
                current_command = action.command
                action.status = 'ongoing'

                if(current_command == PostaCommands.SetScaleToZero):
                    self.zero_scale()
                    rospy.loginfo('MT Scale zeroed')
                    rospy.sleep(0.3)
                
                elif(current_command == PostaCommands.GetWeight):
                    rospy.loginfo('Getting weight')
                    self.send_error_msg('STILL TO IMPLEMENT', 'INFO')

                # Scale Final Status
                self.agents_status['scale'] = (action.name, action.status, action.p1, action.p2)

                if len(self.scale_action_list)>0:
                    self.scale_action_list.pop(0)
            
            rospy.sleep(0.5)



            

if __name__ == '__main__':
    v = SUDPostaControllerV2()

    # Initialize gripper and check if it's active, otherwise exit
    if v.activate_gripper_and_check():
        rospy.loginfo('Gripper activate successfully')
    else:
        rospy.logerr('FAILED to activate Gripper!')
        exit
    
    rospy.loginfo('Connecting to client...')

    v.ControllerStart()
    
    