#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from math import *
import time

class ID_control:
    def __init__(self):
        rospy.init_node("ar_marker2")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        rospy.Subscriber("/limo/crosswalk/distance", Int32, self.crosswalk_distance_callback)
        self.pub = rospy.Publisher("/limo/marker/cmd_vel", Twist, queue_size=5)
        self.pub1 = rospy.Publisher("/limo/marker/bool", Bool, queue_size=5)
        self.pub2 = rospy.Publisher("/limo/marker/stop", Bool, queue_size=5)
        self.pub3 = rospy.Publisher("/limo/marker/park", Bool, queue_size=5)
        self.drive_data = Twist()
        self.flag = None
        self.override_twist = False
        self.collect = None
        self.crosswalk_detected = False
        self.crosswalk_distance = 0
        self.stop = False
        self.park_bool = False
        self.kim_distance=0     
        self.start_time = rospy.get_time()
        self.rate = rospy.Rate(5)

    def crosswalk_distance_callback(self, _data):
        if _data.data == -1:
            self.crosswalk_detected = False
            self.crosswalk_distance = _data.data
        else:
            self.crosswalk_detected = True
            self.crosswalk_distance = _data.data
            #print("find")

    def marker_CB(self, data):
        for marker in data.markers:
            kim_x = marker.pose.pose.position.x
            kim_y = marker.pose.pose.position.y
            kim_z = marker.pose.pose.position.z
            self.kim_distance = (kim_x**2+kim_y**2+kim_z**2)**0.5
            # print(self.kim_distance)

            if marker.id == 0:
                self.found_sign("stop")
            elif marker.id == 1:
                self.found_sign("right")
            elif marker.id == 2:
                self.found_sign("left")
            elif marker.id == 3:
                self.found_sign("park")

    def found_sign(self, _data):
        if self.flag == None:
            self.collect = _data
            # rospy.loginfo(f"collect {_data} marker")
            # rospy.loginfo(self.kim_distance)

            if self.crosswalk_detected == False and (_data in ("right", "park")):
                return
            elif self.kim_distance < 0.8 and (_data in ("stop", "left")):
                return
            else:
                self.start_time = rospy.get_time()
                self.flag = self.collect

    def stop_sign(self):
        if self.flag != "stop":
            return

        passed_time = rospy.get_time() - self.start_time
        self.stop = True
        self.pub2.publish(self.stop)
        if passed_time > 2:
            self.flag = None
            self.stop = False
            # rospy.loginfo("STOP Marker End")
        elif passed_time > 0.5:
            self.override_twist = False
        else:
            self.override_twist = True
            self.drive_data.linear.x = 0.0
            self.drive_data.angular.z = 0.0

    def right_turn_sign(self):
        if self.flag == "right":
            passed_time = rospy.get_time() - self.start_time
            if passed_time > 2:
                self.flag = None
                # rospy.loginfo("TURN Marker End")
            elif passed_time > 1.1:
                self.override_twist = False
            elif passed_time > 0.4:
                self.override_twist = True
                self.drive_data.linear.x = 0.3
                self.drive_data.angular.z = -4.0
                direction = 1
        else:
            return

    def left_turn_sign(self):
        if self.flag == "left":
            passed_time = rospy.get_time() - self.start_time 
            if passed_time > 2:
                self.flag = None
                # rospy.loginfo("TURN Marker End")
            elif passed_time > 1.6:
                self.override_twist = False
            elif passed_time > 0.2:
                self.override_twist = True
                self.drive_data.linear.x = 0.3
                self.drive_data.angular.z = 4.0
        else:
            return

    def park_sign(self):
        if self.flag == "park":
            passed_time = rospy.get_time() - self.start_time
            self.park_bool = True
            self.pub3.publish(self.park_bool)

            if passed_time > 3:
                self.flag = None
            elif passed_time > 2.5:
                self.override_twist = False
            elif passed_time > 2:
                self.drive_data.linear.x = -0.3
                self.drive_data.angular.z = 0.0
            elif passed_time > 1.5:
                self.drive_data.linear.x = 0.3
                self.drive_data.angular.z = 0.0
            elif passed_time > 0:
                self.override_twist = True
                self.drive_data.linear.x = 0.0
                self.drive_data.angular.z = -1.0
        else:
            return
    
    def main(self):
        self.stop_sign()
        self.right_turn_sign()
        self.left_turn_sign()
        self.park_sign()
        self.pub.publish(self.drive_data)
        self.pub1.publish(self.override_twist)
        self.rate.sleep()

if __name__ == "__main__":
    try:
        id_control = ID_control()
        while not rospy.is_shutdown():
            id_control.main()
    except rospy.ROSInterruptException:
        pass
