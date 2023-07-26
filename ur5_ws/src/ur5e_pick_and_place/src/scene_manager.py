#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs
import moveit_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def loadScene(targetSceneName, scene):
    if targetSceneName == "mainScene":
        loadMainScene(scene)
        
def loadMainScene(scene):
    referenceFrameId = "frame_base_link"
    """
    Scene objects needed
        - Base stand
        - Tile holder
        - Tile rack
    """
    #Create and attach base_stand
    newObjectToScene(referenceFrameId,scene,"base_stand",[0.715,0.7,0.025],[0,0.30,0.0326])
    scene.attach_box(referenceFrameId,"base_stand")
    
    #Create and attach tile_holder
    newObjectToScene(referenceFrameId,scene,"tile_holder",[0.135,0.265,0.03],[0.30,0.52,0.058])
    scene.attach_box(referenceFrameId,"tile_holder")
    
    #Create and attach tile_holder
    newObjectToScene(referenceFrameId,scene,"tile_rack",[0.34,0.34,0.29],[-0.19,0.48,0.19])
    scene.attach_box(referenceFrameId,"tile_rack")

    
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
    
    
   
    