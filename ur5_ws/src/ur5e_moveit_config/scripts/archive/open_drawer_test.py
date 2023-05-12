#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# To control the robotiq gripper
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from time import sleep
import time

def ArrayToPose(v):
    #print 'Original waypoint array: ',v
    msg = Pose()
    msg.position = Point(x=v[0], y=v[1], z=v[2])
    msg.orientation = Quaternion(w=v[3], x=v[4], y=v[5], z=v[6])
    #print 'Transformed pose: ',msg
    return msg

class OpenDrawer:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('open_drawer', anonymous=True)
        rospy.loginfo('Main Node [open_drawer] initialized')

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
        group_name = "ur5_arm" # this should be the same name used in RViz 

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        # Robotiq Gripper
        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size=10)
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        rospy.Subscriber("Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, self.getRobotiqGripperStatus)
        self.gripper_in_position = False

        rospy.loginfo('OpenDrawer initialized')

        self.rest = [0.570106, -0.013876, 0.99947, -0.704092, 0.710104, 0.0016, 0.001944]
        self.drawer_half_cabinet = [0.492221, -0.00229, 0.62381, -0.004201, -0.939005, -0.000456, 0.343877]
        self.drawer6_before_handle = [0.396826, -0.002125, 0.355589, -0.00397, -0.938907, -3.3e-05, 0.344149]
        self.drawer6_at_handle = [0.375927, -0.002047, 0.315644, -0.004008, -0.938913, -8.3e-05, 0.344132]
        self.drawer6_half_open = [0.650745, -0.000349, 0.316049, -0.003554, -0.939239, -0.002477, 0.343237]
        self.drawer6_half_open_up = [0.650745, -0.000349, 0.6, -0.003554, -0.939239, -0.002477, 0.343237]

    def getRobotiqGripperStatus(self,status):
        if(status.gOBJ == 0):
            #rospy.loginfo('Fingers are in motion (only meaningful if gGTO = 1)')
            self.gripper_in_position = False
        if(status.gOBJ == 1):
            # rospy.loginfo('Fingers have stopped due to a contact while opening')
            self.gripper_in_position = True
        if(status.gOBJ == 2):
            #rospy.loginfo('Fingers have stopped due to a contact while closing')
            self.gripper_in_position = True
        if(status.gOBJ == 3):
            #rospy.loginfo('Fingers are at requested position')
            self.gripper_in_position = True
        s = "Gripper in position {0}".format(self.gripper_in_position)
        #rospy.loginfo(s)

    def activateGripper(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output();
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP  = 255
        self.command.rFR  = 150
        self.robotiq_pub.publish(self.command)
        rospy.loginfo('Gripper activation message sent')

    def moveGripperAndWait(self, gripper_p):
        msg = "MoveGripperAndWait TO: {0}...".format(gripper_p)
        rospy.loginfo(msg)
        # gripper_p 0=open, 255=close
        self.command.rPR = gripper_p
        self.robotiq_pub.publish(self.command)
        # wait until it's at position (but put a limit on the time)
        start_time = time.time()
        while not (self.gripper_in_position):
            rospy.sleep(0.1)
            end_time = time.time()
            duration = end_time-start_time
            if(duration>5): # in seconds
                msg = "Too long to reach gripper position ({0}secs), has it been activated??".format(duration)
                raise Exception(msg)
        rospy.loginfo("   DONE, gripper in position")
        return

    def execute_cartesian_path(self, waypoints):
        mg = self.move_group
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        mg.execute(plan, wait=True)
        
    def openDrawer6(self):
        
        mg = self.move_group

        # move gripper to open
        self.moveGripperAndWait(0)

        # start from current pose and go to the approach
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer_half_cabinet))
        waypoints.append(ArrayToPose(self.drawer6_before_handle))
        self.execute_cartesian_path(waypoints)

        # move gripper to engage position
        self.moveGripperAndWait(120)

        # engage the handle
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_at_handle))
        self.execute_cartesian_path(waypoints)

        # open the drawer
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open))
        self.execute_cartesian_path(waypoints)

        # release the handle
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open_up))
        self.execute_cartesian_path(waypoints)

        # move gripper to open
        self.moveGripperAndWait(0)
        

def main():
    v = OpenDrawer()
    v.activateGripper()
    v.openDrawer6()

    rospy.spin()


if __name__ == '__main__':
    main()
        



