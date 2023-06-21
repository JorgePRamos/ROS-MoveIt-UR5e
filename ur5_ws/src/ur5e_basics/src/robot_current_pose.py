#!/usr/bin/env python

def read_current_pose(moveGroup):

    current_pose = moveGroup.get_current_pose().pose
    return current_pose

def ArrayToPose(targetArray):
    resultingPose = Pose()
    resultingPose.position.x = targetArray[0]
    resultingPose.position.y = targetArray[1]
    resultingPose.position.z = targetArray[2]
    resultingPose.orientation.x = targetArray[3]
    resultingPose.orientation.y = targetArray[4]
    resultingPose.orientation.z = targetArray[5]
    resultingPose.orientation.w = targetArray[6]
    return resultingPose

def cartesianMoveToPose(moveGroup, targetPose):
    waypoints = []
    waypoints.append(ArrayToPose(targetPose))

    #TODO check last two arguments of compute_cartesian
    (plan, fraction) = moveGroup.compute_cartesian_path(waypoints,0.005,0.0)
    moveGroup.execute(plan, wait=True)



if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5e_current_pose', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    #Set MoveIt arm group
    group_name = "ur5e_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)


    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)