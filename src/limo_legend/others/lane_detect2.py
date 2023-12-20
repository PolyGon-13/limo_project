#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/lane_detect.py
# WeGo LIMO Pro를 이용한 차선 인식 코드

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32
from dynamic_reconfigure.server import Server
from limo_application.cfg import image_processingConfig
import cv2
import numpy as np

class LaneDetection:
    '''
        ROS 기반 차선 인식 객체
        Private Params --> image_topic_name, visualization
        Image Topic Subscriber (CompressedImage Type)
        Distance to Left Lane Topic Publisher (Int32 Type)
    '''
    def __init__(self):
        # ROS Part
        rospy.init_node("lane_detect")
        srv = Server(image_processingConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
        self.distance_pub = rospy.Publisher("/limo/lane_x", Int32, queue_size=5)
        self.viz = rospy.get_param("~visualization", False)
    
    # return np.ndarray (opencv image type)
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        '''
            원하는 이미지 영역 검출
        '''
        i2=cv2.blur(_img[420:480, 0:320], (41, 41))   #짝수는 안됨
        #i2=cv2.GaussianBlur(_img[420:480, 0:320], (41, 41), sigmaX = 0, sigmaY = 0)
        return i2
    
    # return np.ndarray (opencv image type)
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        '''
            특정 색 영역만 추출 (Dynamic Reconfigure를 통해, 값 변경 가능)
        '''
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLOW_LANE_LOW_TH, self.YELLOW_LANE_HIGH_TH)
        return mask_yellow
    
    # return Int
    def calcLaneDistance(self, _img=np.ndarray(shape=(480, 640))):
        '''
            최종 검출된 이미지를 이용하여 차선의 모멘트 계산
            모멘트의 x, y 좌표 중 차량과의 거리에 해당하는 x를 반환
        '''
        try:
            M = cv2.moments(_img)
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])
        except:
            self.x = -1
            self.y = -1
        # print("x, y = {}, {}".format(x, y))
        return self.x * 1.2  #수정한 부분---------------------------------------------------
    
    def visResult(self):
        '''
            최종 결과가 추가된 원본 이미지 (lane_original)
            차선 영역만 ROI로 잘라낸 이미지 (lane_cropped)
            ROI 내부 중 특정 색 영역만 검출한 이미지 (lane_threshold)
        '''
        '''cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        cv2.imshow("lane_original", self.frame)
        cv2.imshow("lane_cropped", self.cropped_image)'''
        cv2.imshow("lane_thresholded", self.thresholded_image)
        cv2.waitKey(1)
    
    # ==============================================
    #               Callback Functions
    # ==============================================
    
    def reconfigure_callback(self, config, level):
        '''
            Dynamic_Reconfigure를 활용하여, 차선 검출을 위한 색 영역 지정
            HLS Color Space를 기반으로 검출
            노란색 차선을 검출을 위한 Threshold 설정
        '''
        self.YELLOW_LANE_LOW_TH = np.array([config.yellow_h_low, config.yellow_l_low, config.yellow_s_low]) 
        self.YELLOW_LANE_HIGH_TH = np.array([config.yellow_h_high, config.yellow_l_high, config.yellow_s_high])
        return config

    def image_topic_callback(self, img):
        '''
            실제 이미지를 입력 받아서 동작하는 부분
            CompressedImage --> OpenCV Type Image 변경 (compressed_imgmsg_to_cv2)
            차선 영역만 ROI 지정 (imageCrop)
            ROI 영역에서 차선 색 영역만 검출 (colorDetect)
            검출된 차선을 기반으로 거리 계산 (calcLaneDistance)
            최종 검출된 값을 기반으로 카메라 좌표계 기준 차선 무게중심 점의 x좌표 Publish
        '''
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.left_distance = self.calcLaneDistance(self.thresholded_image)
        self.distance_pub.publish(self.left_distance)

        # visualization
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
