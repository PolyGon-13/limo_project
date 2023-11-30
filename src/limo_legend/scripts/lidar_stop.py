#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

class LidarObjectDetector:
    def __init__(self):
        rospy.init_node('lidar_stop', anonymous=False)
        self.warn_pub = rospy.Publisher("/limo/lidar_warn", String, queue_size=3)
        self.sub_ls = rospy.Subscriber(rospy.get_param("lidar_topic_name", "/scan"), LaserScan, self.lidar_callback)
        self.E_STOP_MIN_ANGLE_DEG = -10.0
        self.E_STOP_MAX_ANGLE_DEG = 10.0
        self.E_STOP_DISTANCE_METER = 0.3
        self.E_STOP_COUNT = 5

        self.USE_LIFT = True
        if self.USE_LIFT:
            self.lifter_ctrl_pub = rospy.Publisher("/chatter_updown", UInt8, queue_size=3)
            self.Warning_Status = False
            self.Warning_first_time = rospy.Time.from_sec(0.0)
            self.Safe_first_time = rospy.Time.now()

    def lidar_callback(self, _data):
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
        if rospy.Time.now().to_sec() - self.Warning_first_time.to_sec() > 5.0:
            self.lifter_ctrl_pub.publish(1)
        elif 5.0 < rospy.Time.now().to_sec() - self.Safe_first_time.to_sec() < 6.0:
            self.lifter_ctrl_pub.publish(2)

def run():
    new_class = LidarObjectDetector()
    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
