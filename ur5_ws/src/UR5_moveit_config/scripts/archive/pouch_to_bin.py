#!/usr/bin/env python

import sys
import re
import copy
import rospy
import moveit_commander
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# To control the robotiq gripper
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')

from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

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

class PouchToBin:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('sud_posta_controller', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm" # this should be the same name used in RViz 
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', 
            outputMsg.Robotiq2FGripper_robot_output,
            queue_size=10)
        self.command = outputMsg.Robotiq2FGripper_robot_output()

        bbox_topic = "/sud_detected_pouches_location"
        self.bounding_boxes = None
        self.bounding_boxes_header = None
        rospy.Subscriber(bbox_topic, BoundingBoxes, self.pouches_bbox_callback)

        # Get the robot state
        self.robot_state = self.robot.get_current_state()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.counter = 0

        self.distance_between_drawers = 0.115
        
        # Store positions for drawer 6
        self.drawer6_shoot_image = [0.568175, 0.001095, 1.09218, -0.704762, 0.709434, 0.002133, 0.003082]
        self.drawer6_pickup_center = [0.69133, 0.008065, 0.538871, -0.697368, 0.716553, 0.001512, 0.01512]
        self.drawer6_ul_plate = [0.852798, 0.280818, 0.350601, -0.693819, 0.719806, 0.005126, 0.021616]
        self.to_bin_1 = [0.488656, -0.206905, 1.220312, -0.705791, 0.708406, -0.000681, 0.004386]
        self.to_bin_2 = [0.385918, -0.309599, 1.324864, -0.706453, 0.707745, -0.002492, 0.00379]
        self.on_bin = [0.244225, -0.354504, 1.324834, -0.706391, 0.707807, -0.003797, 0.002513]
        self.maxAgeForBoxesRecognitionSecs = 10

        self.xmin = 0.45
        self.xmax = 0.85
        self.ymin = -0.26
        self.ymax = 0.26
        self.xmid = (self.xmin + self.xmax)/2
        self.ymid = (self.ymin + self.ymax)/2
        self.zref = 0.5
        self.drawer6plateZ = 0.35

        self.RPY_UL = [-142,0,-35]
        self.RPY_BL = [-134,0,49]
        self.RPY_UR = [-157,0,-40]
        self.RPY_BR = [-136,0,83]
        self.RPY_CNT = [-148,0,-4]
        self.RPY_0 = [180,0,-90]

        # print to log
        rospy.loginfo('INIT completed')

    def pouches_bbox_callback(self, data):
        self.bounding_boxes = data.bounding_boxes
        self.bounding_boxes_header = data.header

    def go_to_drawer_imaging_position(self, drawer_number):
        mg = self.move_group        
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        delta_z = self.distance_between_drawers * (6 - drawer_number)
        pose_shoot = ArrayToPose(self.drawer6_shoot_image,delta_z) 
        waypoints.append(pose_shoot)
        self.execute_cartesian_path(waypoints,0.05)

    def go_to_position(self, pose_array):
        mg = self.move_group        
        rospy.loginfo('   GO toPose {},{},{}'.format(pose_array[0],
            pose_array[1], pose_array[2]))
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(pose_array,0))
        self.execute_cartesian_path(waypoints,0.05)

    def go_to_pick_position(self, pose_array):
        mg = self.move_group        
        rospy.loginfo('   GO toPose {},{},{}'.format(pose_array[0],
            pose_array[1], pose_array[2]))
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_pickup_center,0))
        waypoints.append(ArrayToPose(pose_array,0))
        self.execute_cartesian_path(waypoints,0.05)

    def go_to_bin(self):
        mg = self.move_group        
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.to_bin_1,0))
        waypoints.append(ArrayToPose(self.to_bin_2,0))
        waypoints.append(ArrayToPose(self.on_bin,0))
        self.execute_cartesian_path(waypoints,0.05)

    def go_from_bin_to_imaging(self):
        mg = self.move_group        
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.to_bin_2,0))
        waypoints.append(ArrayToPose(self.to_bin_1,0))
        waypoints.append(ArrayToPose(self.drawer6_shoot_image,0))
        self.execute_cartesian_path(waypoints,0.05)

    def generate_interpolated_pose(self, x, y, z):
        # given an xy location on the plate, generate the interpolated pose
        # Interpolate along the upper and lower horizontal border
        rUDeg = interpolate_in_array(self.RPY_UR,self.RPY_UL,self.ymin,self.ymax,y) 
        rBDeg = interpolate_in_array(self.RPY_BR, self.RPY_BL,self.ymin,self.ymax,y) 
        # Interpolate along the vertical direction
        rFDeg = interpolate_in_array(rBDeg,rUDeg,self.xmin,self.xmax,x)
        # Convert to radians
        rF = [rFDeg[0]*pi/180, rFDeg[1]*pi/180, rFDeg[2]*pi/180]
        # Transform to quaternion
        qF = quaternion_from_euler(rF[0],rF[1],rF[2])
        # Generate the output pose
        pOut = [x,y,z,qF[0],qF[1],qF[2],qF[3]]
        return pOut

    def execute_cartesian_path(self, waypoints, speed_scaling):    
        mg = self.move_group
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)
        mg.execute(plan, wait=True)
        return

    def activate_gripper(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.robotiq_pub.publish(self.command)
        rospy.sleep(2)
        return

    def open_gripper(self):
        self.command.rPR = 0
        self.robotiq_pub.publish(self.command)
        rospy.sleep(2)
        return

    def close_gripper(self):
        self.command.rPR = 255
        self.robotiq_pub.publish(self.command)
        rospy.sleep(2)
        return

    def set_gripper_aperture(self, aperture_mm):
        x_command = (86.6-aperture_mm)/0.38
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
        rospy.sleep(2)
        return

    def start(self):
        rospy.loginfo('SUD Pouch to Bin START')
        while not rospy.is_shutdown():
            # are there pouches to pickup, from not too long ago?

            detection_age_msec = 0
            nBBoxes = 0
            if(self.bounding_boxes_header is not None):
                detection_age_sec = (rospy.get_rostime() - self.bounding_boxes_header.stamp).to_sec()
                
            if self.bounding_boxes is not None:
                nBBoxes = len(self.bounding_boxes)

            if (self.bounding_boxes is not None) and (detection_age_sec>0) and (detection_age_sec<(self.maxAgeForBoxesRecognitionSecs)):
                self.counter = self.counter+1
                rospy.loginfo('CNT[{}] Pouch Detected'.format(self.counter))

                # pick the first pouch in the list
                bbox = self.bounding_boxes[0]
                # transform to the pickup point center (in image )
                pPouch = (
                    (bbox.xmin + bbox.xmax)/2.0/1000.0,
                    (bbox.ymin + bbox.ymax)/2.0/1000.0
                )
                rospy.loginfo('pPickup = {},{}'.format(pPouch[0],pPouch[1]))
                # define the pickup pose
                pose_pickup_high = self.generate_interpolated_pose(pPouch[0], pPouch[1], self.drawer6plateZ + 0.15)
                # Go the pose high
                rospy.loginfo('CNT[{}] Going to pouch pickup high'.format(self.counter))
                rospy.loginfo('  p[{},{},{}]'.format(
                    pose_pickup_high[0],pose_pickup_high[1],pose_pickup_high[2]))
                self.go_to_position(pose_pickup_high)

                # Open the gripper
                self.set_gripper_aperture(60) # in mm
                
                # Go to the pickup pose
                pose_pickup = self.generate_interpolated_pose(pPouch[0], pPouch[1], self.drawer6plateZ+0.005)
                # Go to the pickup position
                rospy.loginfo('CNT[{}] Going to pouch PICKUP [{},{},{}]'.format(self.counter, pose_pickup[0], pose_pickup[1], pose_pickup[2]))
                self.go_to_position(pose_pickup)
                # Close the gripper
                self.set_gripper_aperture(20)
                # Go the pose high
                self.go_to_position(pose_pickup_high)
                # Go to bin
                rospy.loginfo('CNT[{}] Going to BIN'.format(self.counter))
                self.go_to_bin()
                # Open the gripper
                self.set_gripper_position(100)
                # Go back to Imaging
                self.go_from_bin_to_imaging()
                rospy.loginfo('CNT[{}] Completed!'.format(self.counter))
                rospy.loginfo('--------------------------------------'.format(self.counter))
                rospy.sleep(4)
            else:
                msg = 'Cannot START --> detectionAge[{}] nBBoxes[{}]'.format(detection_age_msec,nBBoxes)
                rospy.loginfo(msg)
                rospy.sleep(0.5)
      

if __name__ == '__main__':
    v = PouchToBin()

    # go to the Drawer6 view point
    v.go_to_drawer_imaging_position(6)
    v.activate_gripper()

    v.start()


