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
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def ArrayToPose(v):
    #print 'Original waypoint array: ',v
    msg = Pose()
    msg.position.x = v[0]
    msg.position.y = v[1]
    msg.position.z = v[2]
    msg.orientation.x = v[3]
    msg.orientation.y = v[4]
    msg.orientation.z = v[5]
    msg.orientation.w = v[6]
    return msg

def RadToDeg(v):
    return v * 180.0 / pi

class NamedPose:
    def __init__(self, name, pose):
        self.Index = -1
        self.Name = name
        self.Pose = pose

class SavePoses:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5_pose_saver', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur5_arm" # this should be the same name used in RViz 
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # Get the robot state
        #robot_state = robot.get_current_state()
        #rospy.loginfo(robot_state)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Initialize properties
        self.filename = "/home/bicrobotics/ur5ws/ee_poses.txt"
        self.poses = []
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.read_poses_from_file()

        # print to log
        rospy.loginfo('INIT completed')

    def read_current_pose(self):
        move_group = self.move_group
        current_pose = move_group.get_current_pose().pose
        return current_pose

    def SortPoses(self):
        self.poses.sort(key=lambda x: x.Name)
        cnt = 0
        for p in self.poses:
            cnt += 1
            p.Index = cnt

    def move_to_pose_cartesian(self, pose):
        mg = self.move_group
        waypoints = []
        # waypoints.append(copy.deepcopy(mg.get_current_pose().pose))
        waypoints.append(ArrayToPose(pose))
        (plan, fraction) = mg.compute_cartesian_path(waypoints,0.005,0.0)
        mg.execute(plan, wait=True)

    def read_poses_from_file(self):
        self.poses = []
        infile = open(self.filename,'r')
        lines = infile.readlines()

        cnt = 0
        for line in lines:
            cnt += 1
            x = re.search("\[.+\]", line)
            if x:
                name = line.split(' ')[0]
                p = x.group(0)
                p = p.replace("[","").replace("]","")
                values = p.split(',')
                pose = [float(v) for v in values]
                named_pose = NamedPose(name,pose)
                self.poses.append(named_pose)

        # reorder poses
        self.SortPoses()

        return

    def print_poses(self):
        for p in self.poses:
            print('[P{0}] - {1}'.format(p.Index,p.Name))

    

    def print_poses_rpy(self):
        for p in self.poses:
            rpy = euler_from_quaternion(p.Pose[3:])
            print('P{} = {},{},{}'.format(p.Name, RadToDeg( rpy[0]), RadToDeg(rpy[1]), RadToDeg(rpy[2])))

    def read_current_pose_as_array(self):
        move_group = self.move_group
        p = move_group.get_current_pose().pose
        n = 6
        return [
            round(p.position.x,n), 
            round(p.position.y,n), 
            round(p.position.z,n), 
            round(p.orientation.x,n), 
            round(p.orientation.y,n), 
            round(p.orientation.z,n), 
            round(p.orientation.w,n)
            ]

    def get_pose_with_index(self, index):
        for p in self.poses:
            if(p.Index==index):
                return p
        return None

    def add_pose(self, name, pose):
        self.poses.append(NamedPose(name,pose))
        self.SortPoses()

def main():
    try:
        v = SavePoses()
        val = ""

        # print all poses
        v.print_poses()

        while val!="quit":

            val = raw_input('Enter pose name or [quit,list,rpy,<num_pose>]: ')

            if((val!='quit') & (len(val)>0)):
                
                # Check if it is a command or a name of a pose
                
                if(val=='list'):
                    # list all the poses in the file
                    v.print_poses()
                
                elif re.search('^P\d+',val):
                    pose_index = int(re.search('^P\d+',val).group(0).replace("P",""))
                    pose = v.get_pose_with_index(pose_index)
                    if(pose):
                        print('Pose[{0}] - {1} REQUESTED'.format(pose.Index,pose.Name))
                        v.move_to_pose_cartesian(pose.Pose)
                    else:
                        print('ERROR: no pose found with index {0}'.format(pose_index))

                elif(val=="rpy"):
                    # print all the poses with angles in RPY
                    v.print_poses_rpy()

                else:
                    outfile = open(v.filename, "a")
                    pose = v.read_current_pose_as_array()
                    pose_text = val + ' = ' + str(pose)
                    outfile.write(pose_text + '\n')
                    outfile.close()
                    v.add_pose(val,pose)
                    print ('Saved pose: ' + pose_text)

    except Exception as e:
        print('EXCEPTION!' + str(e))
        return


if __name__ == '__main__':
    main()

