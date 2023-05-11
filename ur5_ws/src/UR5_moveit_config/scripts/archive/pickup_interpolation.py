#!/usr/bin/env python

import sys
import re
import copy
import rospy
import moveit_commander
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

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

class PickupInterpolator:

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

        # Get the robot state
        self.robot_state = self.robot.get_current_state()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.counter = 0

        self.distance_between_drawers = 0.115
        
        # Store positions for drawer 6
        
        self.drawer6_pickup_center = [0.69133, 0.008065, 0.538871, -0.697368, 0.716553, 0.001512, 0.01512]
        
        self.pick6UL = [0.843577, 0.257424, 0.375136, -0.900007, 0.2969, -0.079083, 0.309168]
        self.pick6UR = [0.838361, -0.262808, 0.360605, -0.923618, 0.327853, -0.106283, 0.167769]

        self.pick6BL = [0.463432, 0.269807, 0.365933, -0.842268, -0.37385, 0.177815, 0.345259]
        self.pick6BR = [0.448591, -0.258457, 0.359744, -0.685311, -0.627863, 0.21637, 0.298867]
        
        self.pick6MidL = [0.637266, 0.264054, 0.374943, -0.936201, 0.046596, -0.004349, 0.348335]
        self.pick6Center = [0.648819, -0.016092, 0.365684, -0.961049, 0.042829, 0.020099, 0.272298]
        self.pick6BMid = [0.463452, -0.024807, 0.367569, -0.824846, -0.385514, 0.167637, 0.378029]
        self.pick6UMid = [0.839776, -0.006139, 0.376185, -0.857784, 0.323739, -0.139871, 0.373946]
        self.pick6MidR = [0.652402, -0.258386, 0.365497, -0.979014, 0.007779, 0.033515, 0.200866]
        self.pick6Straight = [0.684737, 0.003299, 0.566452, 0.704236, -0.709965, -0.000362, 0.001317]

        self.UR_euler = euler_from_quaternion(self.pick6UR[3:7])
        self.UMid_euler = euler_from_quaternion(self.pick6UMid[3:7])
        self.UL_euler = euler_from_quaternion(self.pick6UL[3:7])
        self.BR_euler = euler_from_quaternion(self.pick6BR[3:7])
        self.BMid_euler = euler_from_quaternion(self.pick6BMid[3:7])
        self.BL_euler = euler_from_quaternion(self.pick6BL[3:7])
        self.Cnt_euler = euler_from_quaternion(self.pick6Center[3:7])
        self.MidL_euler = euler_from_quaternion(self.pick6MidL[3:7])
        self.MidR_euler = euler_from_quaternion(self.pick6MidR[3:7])
        self.Straight_euler = euler_from_quaternion(self.pick6Straight[3:7])

        rospy.loginfo('UR Roll,Pitch.Yaw DEG = {},{},{}'.format(self.UR_euler[0]*180.0/pi,self.UR_euler[1]*180.0/pi,self.UR_euler[2]*180.0/pi))
        rospy.loginfo('UMid Roll,Pitch.Yaw DEG = {},{},{}'.format(self.UMid_euler[0]*180.0/pi,self.UMid_euler[1]*180.0/pi,self.UMid_euler[2]*180.0/pi))
        rospy.loginfo('UL Roll,Pitch.Yaw DEG = {},{},{}'.format(self.UL_euler[0]*180.0/pi,self.UL_euler[1]*180.0/pi,self.UL_euler[2]*180.0/pi))
        rospy.loginfo('BR Roll,Pitch.Yaw DEG = {},{},{}'.format(self.BR_euler[0]*180.0/pi,self.BR_euler[1]*180.0/pi,self.BR_euler[2]*180.0/pi))
        rospy.loginfo('BMid Roll,Pitch.Yaw DEG = {},{},{}'.format(self.BMid_euler[0]*180.0/pi,self.BMid_euler[1]*180.0/pi,self.BMid_euler[2]*180.0/pi))
        rospy.loginfo('BL Roll,Pitch.Yaw DEG = {},{},{}'.format(self.BL_euler[0]*180.0/pi,self.BL_euler[1]*180.0/pi,self.BL_euler[2]*180.0/pi))
        rospy.loginfo('Cnt Roll,Pitch.Yaw DEG = {},{},{}'.format(self.Cnt_euler[0]*180.0/pi,self.Cnt_euler[1]*180.0/pi,self.Cnt_euler[2]*180.0/pi))
        rospy.loginfo('MidL Roll,Pitch.Yaw DEG = {},{},{}'.format(self.MidL_euler[0]*180.0/pi,self.MidL_euler[1]*180.0/pi,self.MidL_euler[2]*180.0/pi))
        rospy.loginfo('MidR Roll,Pitch.Yaw DEG = {},{},{}'.format(self.MidR_euler[0]*180.0/pi,self.MidR_euler[1]*180.0/pi,self.MidR_euler[2]*180.0/pi))
        rospy.loginfo('Straight Roll,Pitch.Yaw DEG = {},{},{}'.format(self.Straight_euler[0]*180.0/pi,self.Straight_euler[1]*180.0/pi,self.Straight_euler[2]*180.0/pi))

        self.xmin = 0.45
        self.xmax = 0.85
        self.ymin = -0.26
        self.ymax = 0.26
        self.xmid = (self.xmin + self.xmax)/2
        self.ymid = (self.ymin + self.ymax)/2
        self.zref = 0.5

        self.RPY_UL = [-142,0,-35]
        self.RPY_BL = [-134,0,49]
        self.RPY_UR = [-157,0,-40]
        self.RPY_BR = [-136,0,83]
        self.RPY_CNT = [-148,0,-4]
        self.RPY_0 = [180,0,-90]
        
        # print to log
        rospy.loginfo('INIT completed')

    def go_to_position(self, pa):
        mg = self.move_group        
        rospy.loginfo('   GO toPose {},{},{},{},{},{},{}'.format(pa[0],
            pa[1], pa[2], pa[3], pa[4], pa[5], pa[6]))
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(pa,0))
        self.execute_cartesian_path(waypoints,0.05)

    def execute_cartesian_path(self, waypoints, speed_scaling):    
        mg = self.move_group
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)
        mg.execute(plan, wait=True)
        return

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

    def generate_pose(self, x,y,z,rollDeg,pitchDeg,yawDeg):
        # convert to radians
        roll = rollDeg * pi/180
        pitch = pitchDeg*pi/180
        yaw = yawDeg*pi/180
        q = quaternion_from_euler(roll,pitch,yaw)
        rospy.loginfo('Generated Quaternion = {}  {}  {}  {}'.format(q[0],q[1],q[2],q[3]))
        pose = [x,y,z,q[0],q[1],q[2],q[3]]
        return pose

    def start(self):
        rospy.loginfo('SUD Pickup Interpolation START')

        # Interpolate along the upper border
        pC = [self.xmid, self.ymid, self.zref+0.3]
        p0 = [self.xmin, self.ymin, self.zref]
        p1 = [self.xmax, self.ymax, self.zref]
        
        # go to center and wait
        pose_center = self.generate_pose(pC[0],pC[1],pC[2], self.RPY_0[0], self.RPY_0[1], self.RPY_0[2])
        self.go_to_position(pose_center)
        
        # go to p0 and wait
        pose0 = self.generate_interpolated_pose(p0[0],p0[1],p0[2])
        self.go_to_position(pose0)
        rospy.sleep(2)
        inital_time = rospy.get_rostime()
        path_duration_secs = 10

        while not rospy.is_shutdown():
            current_time = rospy.get_rostime()
            current_duration = current_time-inital_time
            sTime = current_duration.to_sec()

            if(sTime<path_duration_secs):
                # Interpolate along the trajectory for the xy point
                pS = interpolate_in_array(p0,p1,0,path_duration_secs,sTime)
                # get the interpolated orientation
                pOut = self.generate_interpolated_pose(pS[0],pS[1],pS[2])
                #rospy.loginfo('Interpolated pose at {} = {}'.format(sTime,pOut))
                # Move to this pose
                self.go_to_position(pOut)

if __name__ == '__main__':
    v = PickupInterpolator()
    v.start()
    