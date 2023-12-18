#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32, Float64
import time

class ID_control:
    def __init__(self):
        rospy.init_node("aruco")
        self.drive_data = Twist() # 주행 데이터를 담을 drive_data를 Twist 메시지로 선언
        self.override_twist = False # aruco marker를 인식했는지 여부를 저장
        self.kim_distance=0  # 수학적으로 계산 마커와의 거리를 저장
        self.flag = None # id값에 해당하는 문자열을 저장
        self.park = False # 주차 마커를 인식했는지 여부를 담는 변수
        self.right_good = False
        self.stop = False
        self.collect = None
        self.start_time = rospy.get_time() # 마커 동작을 수행할 때 딜레이를 주기 위해 마커를 인식한 시점에서의 시간을 저장
        self.rate = rospy.Rate(5) # 1초에 5번 loop를 반복할 수 있도록 rate라는 객체를 생성
        self.gtan = 0 # 두 차선의 기울기를 이용해 차선이 어느 한 쪽으로 치우친 정도를 저장
        self.pub = rospy.Publisher("/limo/marker/cmd_vel", Twist, queue_size=5)
        self.pub1 = rospy.Publisher("/limo/marker/bool", Bool, queue_size=5)
        self.park_bool_pub = rospy.Publisher("/limo/marker/park", Bool, queue_size=5)
        self.stop_bool_pub = rospy.Publisher("/limo/marker/stop", Bool, queue_size=5)
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        rospy.Subscriber("/limo/lane/gtan", Float64, self.global_gtan)

    # lane_detect.py로부터 받아온 두 차선의 기울어진 정도에 따른 값을 받아옴
    def global_gtan(self, _data):
        self.gtan = _data.data
        # print(self.gtan)
    
    # 인식한 마커와의 거리를 계산하고, 인식한 마커의 id값에 따른 문자열을 found_sign 함수에 전달
    def marker_CB(self, data):
        for marker in data.markers:
            kim_x = marker.pose.pose.position.x
            kim_y = marker.pose.pose.position.y
            kim_z = marker.pose.pose.position.z
            self.kim_distance = (kim_x**2+kim_y**2+kim_z**2)**0.5 # 마커와의 거리 계산
            # print(self.kim_distance)

            if marker.id == 0:
                self.found_sign("stop")
            elif marker.id == 1:
                if self.gtan > -0.5 and self.right_good == False:
                    self.found_sign("right")
                if self.right_good == True:
                    self.found_sign("right2")
            elif marker.id == 3:
                self.found_sign("park")
    
    # 전달받은 문자열을 저장하고 특정 조건을 만족하면 동작 수행
    def found_sign(self, _data):
        self.collect = _data
        if self.flag == None: # 전달받은 마커 데이터가 없거나, 마커 동작 수행을 끝마쳐 self.flag에 아무 데이터가 없는 경우
            if self.kim_distance > 0.8 and _data == "park": # 마커와의 거리가 0.8보다 큰데 park 신호가 왔을 경우
                return
            else:
                self.start_time = rospy.get_time()
                self.flag = self.collect # 저장해둔 문자열을 self.flag로 전달

    # 0번 마커(정지 신호)를 인식하였다면 아래의 동작 수행
    def stop_sign(self):
        if self.flag != "stop": # main함수에 의해 계속 실행되므로 stop 신호가 아니면 패스
            return

        passed_time = rospy.get_time() - self.start_time # 마커 동작을 수행한 시점으로부터 지난 시간
        if passed_time > 6:
            self.flag = None # 다음 마커 동작 수행을 위해 self.flag 초기화
            # rospy.loginfo("STOP Marker End")
        elif passed_time > 0.5:
            self.override_twist = False # control.py에 마커 동작 수행이 끝났음을 알려줄 변수를 False로 전환
        else:
            # print("stop_start")
            self.stop = True
            self.override_twist = True # control.py에 마커 동작 수행이 끝났음을 알려줄 변수를 True로 전환
            self.drive_data.linear.x = 0.0
            self.drive_data.angular.z = 0.0

    # 1번 마커(우회전 신호)를 인식하였다면 아래의 동작 수행
    def right_turn_sign(self):
        if self.flag != "right": # main함수에 의해 계속 실행되므로 right 신호가 아니면 패스
            return
        
        passed_time = rospy.get_time() - self.start_time
        if passed_time > 3.7:
            self.flag = None
            self.override_twist = False
            # rospy.loginfo("RIGHT Marker End")
        elif passed_time > 2.2:
            # print("right_start")
            self.override_twist = True
            self.right_good = True
            print("1111")
            self.drive_data.linear.x = 0.0
            self.drive_data.angular.z = -1.0

    # 주차구간 이후 횡단보도 쪽 우회전 마커를 인식할 경우 (gtan를 이용한 연산이 불가능)
    def right2_turn_sign(self):
        if self.flag != "right2":
            return

        passed_time = rospy.get_time() - self.start_time
        if passed_time > 4.3:
            self.flag = None
            self.override_twist = False
            self.park_to_right = False
        elif passed_time > 3.15: # 오른쪽으로 제자리 회전
            self.override_twist = 
            self.right_good = False
            print("2222")
            self.drive_data.linear.x = 0.2
            self.drive_data.angular.z = -1.2

    # 3번 마커(주차 신호)를 인식하였다면 아래의 동작 수행
    def park_sign(self):
        if self.flag != "park": # main함수에 의해 계속 실행되므로 park 신호가 아니면 패스
            return

        passed_time = rospy.get_time() - self.start_time
        if passed_time > 2:
            #self.flag = None # 다음 마커 동작 수행을 위해 self.flag 초기화
            #self.override_twist = False # control.py에 마커 동작 수행이 끝났음을 알려줄 변수를 False로 전환
            self.park = False # 주차 마커 인식 여부를 False로 전환 (다시 가속 가능)
            self.drive_data.linear.x = 0.0
            self.drive_data.angular.z = 0.0
            # rospy.loginfo("PARK Marker End")
        elif passed_time > 1.5: # 조금 직진하여 주차공간에 완벽히 진입
            self.drive_data.linear.x = 0.2
            self.drive_data.angular.z = 0.0
        else: # 적절한 위치에서 우회전하여 주차공간에 진입
            self.override_twist = True
            self.park = True # 주차 마커를 인식했음을 알림 (가속 차단 용도)
            self.drive_data.linear.x = 0.3
            self.drive_data.angular.z = -1.15

    
    # 마커들의 동작을 우선순위를 두어 함수 실행 & 주행 데이터와 마커 인식 유무 데이터 퍼블리시
    def main(self): # 마커 신호에 우선순위를 두었지만 사실 의미가 없다...
        self.right_turn_sign() # 우회전 신호를 3순위로 실행
        self.right2_turn_sign() # 우회전2 신호를 5순위로 실행
        self.park_sign() # 주차 신호를 1순위로 실행
        self.stop_sign() # 정지 신호를 2순위로 실행       
        self.pub.publish(self.drive_data) # 주행 데이터를 퍼블리시
        self.pub1.publish(self.override_twist) # 마커 인식 여부를 담은 변수를 퍼블리시
        self.park_bool_pub.publish(self.park) # 주차 마커 인식 여부를 담은 변수를 퍼블리시
        self.stop_bool_pub.publish(self.stop)
        self.rate.sleep() # 무한루프에서 설정한 주기를 맞추기 위해 기다리는 함수

if __name__ == "__main__":
    try:
        id_control = ID_control()
        while not rospy.is_shutdown():
            id_control.main()
    except rospy.ROSInterruptException:
        pass
