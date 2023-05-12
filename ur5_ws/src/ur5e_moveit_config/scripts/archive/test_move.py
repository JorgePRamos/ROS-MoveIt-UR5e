#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# To control the robotiq gripper
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep

# code copied and adpated from the move_group_python_interface_tutorial.py

class MoveTest:

    def __init__(self):
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_test', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm" # this should be the same name used in RViz 
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.robotiq_pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output)
        self.command = outputMsg.Robotiq2FGripper_robot_output()

        # Get the robot state
        #robot_state = robot.get_current_state()
        #rospy.loginfo(robot_state)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Initialize properties
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        rospy.loginfo('PROPERTIES initialized')

        # print to log
        rospy.loginfo('INIT completed')

    def go_to_pose_cartesian(self):
        
        move_group = self.move_group
        
        waypoints = []
        
        wpose = move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += 0.2
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # # which is why we will specify 0.01 as the eef_step in Cartesian
        # # translation.  We will disable the jump threshold by setting it to 0.0,
        # # ignoring the check for infeasible jumps in joint space, which is sufficient
        # # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

        rospy.loginfo("Cartesian planning DONE")
        rospy.loginfo(plan)

        # Execute the plan
        move_group.execute(plan, wait=True)

        rospy.loginfo("Cartesian execution DONE")

        return

    def activate_gripper(self):
        self.command = outputMsg.Robotiq2FGripper_robot_output()
        self.command.rACT = 1
        self.command.rGTO = 1
        self.command.rSP = 255
        self.command.rFR = 150
        self.robotiq_pub.publish(self.command)
        return

    def open_gripper(self):
        self.command.rPR = 0
        self.robotiq_pub.publish(self.command)
        return

    def close_gripper(self):
        self.command.rPR = 255
        self.robotiq_pub.publish(self.command)
        return


def main():
    try:
        v = MoveTest()

        v.activate_gripper()
        rospy.sleep(3)

        v.open_gripper()
        rospy.sleep(3)

        v.close_gripper()
        rospy.sleep(3)

        v.open_gripper()
        rospy.sleep(3)

        v.go_to_pose_cartesian()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()
