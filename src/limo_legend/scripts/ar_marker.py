#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import *
import time

class ID_control:
    def __init__(self):
        rospy.init_node("ar_marker")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        self.pub = rospy.Publisher("/kim/marker/cmd_vel", Twist, queue_size=10)
        self.pub1 = rospy.Publisher("/kim/marker/bool", Bool, queue_size = 10)
        self.drive_data = Twist()
        self.override_twist = False
        
        self.start_time = rospy.get_time()
        self.rate = rospy.Rate(10)

        self.flag = None

    def marker_CB(self, data):
        for marker in data.markers:
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
            self.start_time = rospy.get_time()
            self.flag = _data
            rospy.loginfo(self.flag)

    def stop_sign(self):
        if self.flag != "stop":
            return

        passed_time = rospy.get_time() - self.start_time
        if passed_time > 2:
            self.flag = None
        elif passed_time > 1:
            self.override_twist = False
        else:
            self.override_twist = True
            self.drive_data.linear.x = 0.0
            self.drive_data.angular.z = 0.0

    def turn_sign(self):
        direction = 0
        if self.flag == "left":
            direction = 1
        elif self.flag == "right":
            direction = -1
        else:
            return

        passed_time = rospy.get_time() - self.start_time      
        if passed_time > 2:
            self.flag = None
        elif passed_time > 1:
            self.override_twist = False
        else:
            self.override_twist = True
            self.drive_data.linear.x = 0.2
            self.drive_data.angular.z = 0.2 * direction

            '''
    def park_sign(self):
        if self.park_flag == False:
            self.park_flag = True
            self.park_time = rospy.get_time()
        else:
            if self.loop_time - self.park_time < 2:
                self.speed = self.basic_speed
                self.angle = -self.basic_angle * pi / 180
            elif self.loop_time - self.park_time < 4:
                self.speed = -self.basic_speed
                self.angle = self.basic_angle * pi / 180
                '''
    def main(self):
        self.stop_sign()
        self.turn_sign()
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
