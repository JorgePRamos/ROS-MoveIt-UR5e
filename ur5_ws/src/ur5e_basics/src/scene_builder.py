#!/usr/bin/env python
"""
Sample code which ilustrates the methods needed to build
a scene using the python interface
"""
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def newObjectToScene(referenceFrameId, scene, newObjectId, dimensions, pose):
    object = CollisionObject()
    object.id = newObjectId
    object.header.frame_id = referenceFrameId

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[0]#X
    object_pose.position.y = pose[1]#Y
    object_pose.position.z = pose[2]#Z

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    scene.add_object(object)


