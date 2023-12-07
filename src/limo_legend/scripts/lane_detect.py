#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Bool
from dynamic_reconfigure.server import Server
from limo_legend.cfg import image_processingConfig
import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detect")
        srv = Server(image_processingConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        self.viz = rospy.get_param("~visualization", True)
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
        self.distance_pub1 = rospy.Publisher("/limo/lane_left", Int32, queue_size=5)
        self.distance_pub2 = rospy.Publisher("/limo/lane_right", Int32, queue_size=5)
        self.lane_connect_pub = rospy.Publisher("/limo/lane_connect", Bool, queue_size=5)
    
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        return _img[420:480, 0:320], _img[420:480, 320:640]
    
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLOW_LANE_LOW_TH, self.YELLOW_LANE_HIGH_TH)
        return mask_yellow
    
    def calcLaneDistance(self, _img=np.ndarray(shape=(480, 640))):
        try:
            M = cv2.moments(_img)
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])
        except:
            self.x = -1
            self.y = -1
        return self.x
    
    def visResult(self):
        cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        cv2.imshow("lane_original", self.frame)
        # cv2.imshow("lane_thresholded_left", self.thresholded_image)
        # cv2.imshow("lane_thresholded_right", self.thresholded_image2)
        cv2.waitKey(1)

    def reconfigure_callback(self, config, level):
        self.YELLOW_LANE_LOW_TH = np.array([config.yellow_h_low, config.yellow_l_low, config.yellow_s_low]) 
        self.YELLOW_LANE_HIGH_TH = np.array([config.yellow_h_high, config.yellow_l_high, config.yellow_s_high])
        return config

    def lane_connect(self, thresholded_image, thresholded_image2):
        connected = np.sum(thresholded_image[:,319]) > 1 and np.sum(thresholded_image2[:,0]) > 1
        self.lane_connect_pub.publish(connected)

    def image_topic_callback(self, img):
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image, self.cropped_image2 = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.thresholded_image2 = self.colorDetect(self.cropped_image2)
        self.left_distance = self.calcLaneDistance(self.thresholded_image)
        self.right_distance = self.calcLaneDistance(self.thresholded_image2)
        self.lane_connect(self.thresholded_image,self.thresholded_image2)
        self.distance_pub1.publish(self.left_distance)
        self.distance_pub2.publish(self.right_distance)
        
        if self.viz:
            self.visResult()
            
def run():
    new_class = LaneDetection()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
