#!/usr/bin/env python

import rospy
import sys  
import math

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Header

import numpy as np
import cv2
import copy
import matplotlib.pyplot as plt
from BirdEyeView import BirdEyeView
from LaneDetector import LaneDetector
from CameraCalibrator import Calibrator

class ClassicLaneDetector:
    def __init__(self ):
        self.bev = BirdEyeView() 
        self.ldt = LaneDetector(self.bev) 
        
        rospy.init_node("DrivingLaneNode")
        self.pub = rospy.Publisher("/lane_map", Image, queue_size=1)
        self.bridge=CvBridge()
        self.sub = rospy.Subscriber("/camera2/usb_cam2/image_raw", Image, self.callback)
        self.image = Image()
        self.lane = Image()
        
        
    def callback(self, data):
        self.image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.lane_detect(self.image)
        cv2.imshow("lane_image", self.lane)
        cv2.waitKey(3)
        
    def lane_detect(self, frame):
        roi_frame=  frame.copy()
        roi_frame=self.bev.setROI(roi_frame)
        warped_frame = self.bev.warpPerspect(frame)
        cv2.imshow("warp", warped_frame)
        cv2.waitKey(3)        
        color_comb_frame = self.bev.combineAllColorChannel(warped_frame,[150,255],[20,100])
        left_fit,right_fit = self.ldt.slidingWindows(color_comb_frame)
        final_frame = self.ldt.drawFitLane(frame, color_comb_frame, left_fit,right_fit)
        self.lane = final_frame
        msg = self.bridge.cv2_to_imgmsg(self.lane)
        msg.header = Header()
        msg.header.frame_id = 'front_cam_imgplane'
        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)
    



def main():
    cld = ClassicLaneDetector()
    try: rospy.spin()
    except KeyboardInterrupt: print("Shutting down")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main()
