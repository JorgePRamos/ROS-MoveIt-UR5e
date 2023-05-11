#!/usr/bin/env python

import sys
import re
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

# Measurements: 460mm is the distance between 4 handles -> 115mm distance between handles of drawers

class OpenDrawer:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_open_drawer', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm" # this should be the same name used in RViz 
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # Get the robot state
        #robot_state = robot.get_current_state()
        #rospy.loginfo(robot_state)
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.distance_between_drawers = 0.115
        
        # Store positions for drawer 6
        self.drawer_raised_position = [0.559734, -0.00245, 1.0618, -0.003348, -0.959885, -0.001748, 0.280369]
        self.drawer_half_cabinet = [0.492221, -0.00229, 0.62381, -0.004201, -0.939005, -0.000456, 0.343877]
        self.drawer6_handle_approach = [0.415184, 0.000814, 0.371352, -0.004018, -0.94294, 4.6e-05, 0.33294]
        self.drawer6_handle_engaged = [0.37, 0.000904, 0.313, -0.003933, -0.94291, 4.3e-05, 0.333023]
        self.drawer6_half_open = [0.674982, 0.002781, 0.316231, -0.003449, -0.943244, -0.002598, 0.332074]
        self.drawer6_half_open_up1 = [0.674837, 0.002211, 0.430359, -0.003724, -0.943191, -0.002088, 0.332224]
        self.drawer6_half_open_up2 = [0.595834, -9.9e-05, 0.42, -0.00393, -0.995784, 0.000587, 0.091647]
        self.drawer6_half_open_reengaged = [0.595831, -7e-06, 0.38239, -0.003999, -0.995782, 0.000489, 0.091662]
        self.drawer6_full_open = [0.930835, 0.000912, 0.377476, 0.00427, 0.996572, 0.001443, 0.082601]
        self.drawer6_full_open_up = [0.930781, 0.000658, 0.424678, 0.004288, 0.996576, 0.001181, 0.08256]
        
        # print to log
        rospy.loginfo('INIT completed')

    def read_current_pose(self):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose
        return current_pose

    def move_to_pose_cartesian(self, pose, speed_scaling):
        mg = self.move_group

        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(pose))

        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)

        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)
        mg.execute(plan, wait=True)
        return

    def execute_cartesian_path(self, waypoints, speed_scaling):
        
        mg = self.move_group
        
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)

        # retime trajectory
        state = self.robot.get_current_state()
        plan = mg.retime_trajectory(state,plan,speed_scaling)

        mg.execute(plan, wait=True)
        return

    def move_to_raised_position(self):
        mg = self.move_group
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer_raised_position))
        self.execute_cartesian_path(waypoints,0.05)

    def open_drawer(self, drawer_number):
        
        # Compute the delta_z to apply to every pose for the specified drawer
        delta_z = self.distance_between_drawers * (6 - drawer_number)

        mg = self.move_group
        
        # 01) go to the approach position and stop
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer_raised_position))
        waypoints.append(ArrayToPose(self.drawer6_handle_approach,delta_z))
        self.execute_cartesian_path(waypoints,0.5)

        #02) engage handle and stop
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_handle_engaged,delta_z))
        self.execute_cartesian_path(waypoints,0.05)

        #03) open drawer until half way
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open,delta_z))
        self.execute_cartesian_path(waypoints,0.05)

        #04) switch to re-engaged position
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_half_open_up1,delta_z))
        waypoints.append(ArrayToPose(self.drawer6_half_open_up2,delta_z))
        waypoints.append(ArrayToPose(self.drawer6_half_open_reengaged,delta_z))
        self.execute_cartesian_path(waypoints,0.05)

        #05) open drawer fully
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_full_open,delta_z))
        self.execute_cartesian_path(waypoints,0.05)

        #06) return to center
        waypoints = []
        waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(self.drawer6_full_open_up,delta_z))
        waypoints.append(ArrayToPose(self.drawer_raised_position))
        self.execute_cartesian_path(waypoints,0.05)


def main():
    try:
        v = OpenDrawer()
        v.open_drawer(6)

    except Exception as e:
        print('EXCEPTION!' + str(e))
        return


if __name__ == '__main__':
    main()

