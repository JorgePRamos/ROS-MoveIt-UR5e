#!/usr/bin/env python

import sys
import re
import os.path
import os
import copy
import rospy
import moveit_commander

import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes

from math import pi
from moveit_commander.conversions import pose_to_list
from cv_bridge import CvBridge, CvBridgeError
import cv2
from cv2 import aruco

class ArucoMarkersLocator:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('aruco_marker_locator', anonymous=True)
        image_topic = "/usb_cam/image_raw"
        rospy.Subscriber(image_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.last_image_cv2 = None

        # Subscribe to Darknet results
        bbox_topic = "/darknet_ros/bounding_boxes/"
        self.bounding_boxes = None
        rospy.Subscriber(bbox_topic, BoundingBoxes, self.darknet_bbox_callback)

        self.search_active = True

        # Publisher of result image
        self.loop_rate = rospy.Rate(5)
        self.aruco_pub = rospy.Publisher('aruco_result_image', Image, queue_size=10)
        self.frame_markers = None
        self.detected_markers_ids = None
        self.detected_markers_corners = None

        # print to log
        rospy.loginfo('INIT completed')

    def darknet_bbox_callback(self, data):
        self.bounding_boxes = data.bounding_boxes
        #for box in data.bounding_boxes:
        #rospy.loginfo('ArUco BBOX {},{},{},{}'.format(box.xmin, box.xmax, box.ymin, box.ymax))
        
    def image_callback(self,msg):
        if(self.search_active):
            try:
                self.last_image_cv2 = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                self.detect_markers()
            except CvBridgeError as e:
                print(e)

    def detect_markers(self):
        gray = cv2.cvtColor(self.last_image_cv2, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()
        self.detected_markers_corners, self.detected_markers_ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.frame_markers = aruco.drawDetectedMarkers(self.last_image_cv2.copy(), 
            self.detected_markers_corners, self.detected_markers_ids)
        
    def start(self):
        rospy.loginfo("ArucoMarkersLocator START")
        while not rospy.is_shutdown():
            if self.frame_markers is not None:
                # Draw the bounding boxes
                if(self.bounding_boxes is not None):
                    for box in self.bounding_boxes:
                        start_point = (box.xmin,box.ymin)
                        end_point = (box.xmax,box.ymax)
                        if(box.Class=="pouch_top"):
                            color = (255,0,100)
                        elif(box.Class=="pouch_bottom"):
                            color = (10,255,0)
                        else:
                            color = (0,0,255)
                        thickness = 2
                        self.frame_markers = cv2.rectangle(self.frame_markers,start_point, end_point, color, thickness)
                self.aruco_pub.publish(self.bridge.cv2_to_imgmsg(self.frame_markers, "bgr8"))
            self.loop_rate.sleep()
    
if __name__ == '__main__':
    my_node = ArucoMarkersLocator()
    my_node.start()