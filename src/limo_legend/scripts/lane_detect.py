#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, Bool
import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detect")
        self.YELLOW_LANE_LOW_TH = np.array([0, 90, 100]) # hls 색 영역의 최솟값 설정
        self.YELLOW_LANE_HIGH_TH = np.array([60, 220, 255]) # hls 색 영역의 최댓값 설정
        self.cvbridge = CvBridge() # 실시간 스트리밍
        self.viz = rospy.get_param("~visualization", True)
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
        self.distance_pub1 = rospy.Publisher("/limo/lane_left", Int32, queue_size=5)
        self.distance_pub2 = rospy.Publisher("/limo/lane_right", Int32, queue_size=5)
        self.lane_connect_pub = rospy.Publisher("/limo/lane_connect", Bool, queue_size=5)
        self.lane_accel_pub = rospy.Publisher("/limo/lane/accel", Bool, queue_size=5)
    
    # 이미지 자르기
    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        return _img[420:480, 0:320], _img[420:480, 320:640]
    
    # 노란색 부분 추출
    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLOW_LANE_LOW_TH, self.YELLOW_LANE_HIGH_TH)
        return mask_yellow
    
    # 차선의 무게중심 계산
    def calcLaneDistance(self, _img=np.ndarray(shape=(480, 640))):
        try:
            M = cv2.moments(_img)
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])
        except:
            self.x = -1
            self.y = -1
        return self.x

    # 왼쪽 차선이 오른쪽 threshold 이미지에 표시될 경우 처리
    def lane_connect(self, thresholded_image, thresholded_image2):
        connected = np.sum(thresholded_image[:,319]) > 1 and np.sum(thresholded_image2[:,0]) > 1
        self.lane_connect_pub.publish(connected)

    # 두 차선이 인식될 경우 True를 반환 (가속 구간)
    def lane_speed(self, thresholded_image, thresholded_image2):
        accel = np.sum(thresholded_image[0,:]) > 1 and np.sum(thresholded_image2[0,:]) > 1
        self.lane_accel_pub.publish(accel)
    
    # 화면에 출력
    def visResult(self):
        cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        # cv2.imshow("lane_original", self.frame)
        cv2.imshow("lane_thresholded_left", self.thresholded_image)
        cv2.imshow("lane_thresholded_right", self.thresholded_image2)
        cv2.waitKey(1)

    def image_topic_callback(self, img):
        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image, self.cropped_image2 = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.thresholded_image2 = self.colorDetect(self.cropped_image2)
        self.left_distance = self.calcLaneDistance(self.thresholded_image)
        self.right_distance = self.calcLaneDistance(self.thresholded_image2)
        self.lane_connect(self.thresholded_image, self.thresholded_image2)
        self.lane_speed(self.thresholded_image, self.thresholded_image2)
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
