#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/e_stop.py
# WeGo LIMO Pro를 이용한 위험 상황 확인 및 긴급 정지 신호 전달 코드

import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
from limo_legend.cfg import lidar_e_stopConfig

import math

class LidarObjectDetector:
    '''
        LiDAR Data 기반으로, 물체가 있는지 여부를 확인하는 객체
        Private Params --> lidar_topic_name
        LiDAR Topic Subscriber (LaserScan Type)
        Warning Topic Publisher (String Type)  
    '''
    def __init__(self):
        rospy.init_node('lidar_stop', anonymous=False)
        srv = Server(lidar_e_stopConfig, self.reconfigure_callback)
        self.sub_ls = rospy.Subscriber(rospy.get_param("lidar_topic_name", "/scan"), LaserScan, self.lidar_callback)
        self.warn_pub = rospy.Publisher("/limo/lidar_warning", String, queue_size=3)
        self.USE_LIFT = True # LIFT 사용 여부 결정 (사용하면 True)
        if self.USE_LIFT:
            self.lifter_ctrl_pub = rospy.Publisher("/chatter_updown", UInt8, queue_size=3)
            self.Warning_Status = False
            self.Warning_first_time = rospy.Time.from_sec(0.0)
            self.Safe_first_time = rospy.Time.now()

    # ==============================================
    #               Callback Functions
    # ==============================================

    def reconfigure_callback(self, _config, _level):
        '''
            Dynamic_Reconfigure를 활용하여, 장애물 검출을 위한 영역 설정
            r, theta 의 극좌표계 기반의 영역 설정
            정면을 0도로 했을 때, 최소 및 최대 측정 각도 (E_STOP_MIN_ANGLE_DEG, E_STOP_MAX_ANGLE_DEG)
            위험하다고 판정할 점의 거리(E_STOP_DISTANCE_METER)
            실제 위험하다고 판단할 위험 영역 내의 점의 개수, 노이즈 방지용(E_STOP_COUNT)
        '''
        self.E_STOP_MIN_ANGLE_DEG = _config.e_stop_min_angle_deg
        self.E_STOP_MAX_ANGLE_DEG = _config.e_stop_max_angle_deg
        self.E_STOP_DISTANCE_METER = _config.e_stop_distance_meter
        self.E_STOP_COUNT = _config.e_stop_count
        return _config

    def lidar_callback(self, _data):
        '''
            실제 라이다 데이터를 받아서 동작하는 부분
            라이다가 측정되는 각도 계산(Radian 및 Degree)
            측정된 데이터 중, 위험 범위 안에 들어 있는 점의 수를 카운팅
            카운팅된 점의 수가 기준 이상이면, 위험 메시지 전달
        '''
        cnt = 0
        angle_rad = [_data.angle_min + i * _data.angle_increment for i, _ in enumerate(_data.ranges)]
        angle_deg = [180 / math.pi * angle for angle in angle_rad]
        for i, angle in enumerate(angle_deg):
            if self.E_STOP_MIN_ANGLE_DEG <= angle <= self.E_STOP_MAX_ANGLE_DEG and 0.0 < _data.ranges[i] < self.E_STOP_DISTANCE_METER:
                cnt += 1
        if cnt >= self.E_STOP_COUNT:
            if self.USE_LIFT:
                if not self.Warning_Status:
                    self.Warning_Status = True
                    self.Warning_first_time = rospy.Time.now()
            self.warn_pub.publish("Warning")
            rospy.logdebug("Object Detected!! Warning!!")
        else:
            if self.USE_LIFT:
                if self.Warning_Status:
                    self.Warning_Status = False
                    self.Safe_first_time = rospy.Time.now()
                self.Warning_first_time = rospy.Time.now()
            self.warn_pub.publish("Safe")
            rospy.logdebug("Safe!!")
        if self.USE_LIFT:
            self.ctrl_lift()
        
    def ctrl_lift(self):
        if rospy.Time.now().to_sec() - self.Warning_first_time.to_sec() > 5.0: # 5초 이상 정지하면 차단기 열기
            self.lifter_ctrl_pub.publish(1)
        elif 5.0 < rospy.Time.now().to_sec() - self.Safe_first_time.to_sec() < 6.0: # 5초 이상, 6초 이하 주행 중이면 차단기 닫기
            self.lifter_ctrl_pub.publish(2)

def run():
    new_class = LidarObjectDetector()
    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
