#!/usr/bin/env python

from copy import deepcopy
import fractions
import sys
import re
import time
import rospy
import moveit_commander
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from std_msgs.msg import String
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO, SetIORequest

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

class RetimeTest():
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('retime_controller', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm" # this should be the same name used in RViz 
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # Get the robot state
        self.robot_state = self.robot.get_current_state()
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.counter = 0

        # positions for drawers opening and closing
        self.p_center = [0.75, 0.0, 0.92, -0.704015, 0.710182, 0.001699, 0.001127]


        # print to log
        rospy.loginfo('INIT completed')

    def generate_pose(self, dx, dy, dz):
        p = deepcopy(self.p_center)
        p[0] = p[0] + dx
        p[1] = p[1] + dy
        p[2] = p[2] + dz
        return p

    def start(self):
        rospy.loginfo('Retime test STARTED')

        scaling = 0.02
        delta_scaling = 0.02

        while not rospy.is_shutdown():
            start_time = rospy.get_rostime()

            rospy.loginfo('Start Motion (scaling={})'.format(scaling))

            waypoints = []
            dX = 0.12
            dY = 0.2

            pUL = self.generate_pose(dX,dY,0)
            pUR = self.generate_pose(-dX,dY,0)
            pBR = self.generate_pose(-dX,-dY,0)
            pBL = self.generate_pose(dX,-dY,0)

            waypoints.append(ArrayToPose(self.p_center ,0))
            waypoints.append(ArrayToPose(pUL ,0))
            waypoints.append(ArrayToPose(pUR ,0))
            waypoints.append(ArrayToPose(pBR ,0))
            waypoints.append(ArrayToPose(pBL ,0))

            mg = self.move_group
            (plan,fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)    
            state = self.robot.get_current_state()
            plan = mg.retime_trajectory(state,plan,scaling)
            response = mg.execute(plan,wait=True)

            end_time = rospy.get_rostime()

            if(response):
                rospy.loginfo('MOVE OK! Duraion={0} at scaling={1}'.format((end_time-start_time).to_sec(), scaling))
            else:
                rospy.logerr('MOVE FAILED')

            scaling += delta_scaling
            if(scaling>1):
                break;



if __name__ == '__main__':
    v = RetimeTest()
    v.start()
