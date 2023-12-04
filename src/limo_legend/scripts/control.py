#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy, roslaunch, os
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from object_msgs.msg import ObjectArray
from limo_base.msg import LimoStatus
from dynamic_reconfigure.server import Server
from limo_legend.cfg import controlConfig
from pathlib import Path
import math

class LimoController:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        self.LIMO_WHEELBASE = 0.2
        self.distance_left = 0
        self.distance_right = 0
        self.e_stop = "Safe"
        self.limo_mode = "ackermann"
        srv = Server(controlConfig, self.reconfigure_callback)
        rospy.Subscriber("limo_status", LimoStatus, self.limo_status_callback)
        rospy.Subscriber("/limo/lane_left", Int32, self.lane_left_callback)
        rospy.Subscriber("/limo/lane_right", Int32, self.lane_right_callback)
        rospy.Subscriber("/limo/lane_connect", Bool, self.lane_connect_callback)
        rospy.Subscriber("/limo/lidar_warn", String, self.lidar_warning_callback)
        rospy.Subscriber("/limo/marker/cmd_vel", Twist, self.marker_cmd_vel_callback)
        rospy.Subscriber("/limo/marker/bool", Bool, self.marker_bool_callback)
        rospy.Subscriber("/imu",Imu, self.imu_callback)
        self.drive_pub = rospy.Publisher(rospy.get_param("~control_topic_name", "/cmd_vel"), Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.drive_callback)
        self.override_twist = False
        self.left_receiveimage = False
        self.lane_connected = False
        self.REF_X2 = 240
        self.launch = dict()

    def roslaunch(self, filename):
        if filename in self.launch:
            return
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        path = Path(os.path.abspath(__file__)).parent.parent.joinpath("launch/%s"%filename)
        self.launch[filename] = roslaunch.parent.ROSLaunchParent(uuid, [str(path)])
        self.launch[filename].start()

    def reconfigure_callback(self, _config, _level):
        self.BASE_SPEED = _config.base_speed
        self.LATERAL_GAIN = float(_config.lateral_gain * 0.001)
        self.REF_X = _config.reference_lane_x
        self.PEDE_STOP_WIDTH = _config.pedestrian_width_min
        return _config
    
    def limo_status_callback(self, _data):
        if _data.motion_mode == 1:
            if self.limo_mode == "ackermann":
                pass
            else:
                self.limo_mode = "ackermann"
                rospy.loginfo("Mode Changed --> Ackermann")
        else:
            if self.limo_mode == "diff":
                pass
            else:
                self.limo_mode = "diff"
                rospy.loginfo("Mode Changed --> Differential Drive")

    def lane_left_callback(self, _data):
        self.left_receiveimage = True
        if _data.data == -1:
            self.distance_left = 0
        else:
            self.distance_left = self.REF_X - _data.data

    def lane_right_callback(self, _data):
        if _data.data == -1:
            self.distance_right = 0
        else:
            self.distance_right = self.REF_X2 - _data.data
            
    def lidar_warning_callback(self, _data):
        self.e_stop = _data.data

    def marker_cmd_vel_callback(self, _data):
        self.new_drive_data = _data

    def marker_bool_callback(self, _data):
        self.override_twist = _data.data

    def lane_connect_callback(self, _data):
        self.lane_connected = _data.data

    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        print(roll_deg,end='  ')
        print(pitch_deg,end='  ')
        print(yaw_deg,end='\n')

    def drive_callback(self, _event):
        drive_data = Twist()
        drive_data.linear.x = self.BASE_SPEED
        drive_data.angular.z = (self.distance_left + self.distance_right) * self.LATERAL_GAIN
        
        if self.override_twist == True:
            new_drive_data = Twist()
            drive_data = self.new_drive_data

        if self.lane_connected == True:
            drive_data.angular.z = self.distance_left * self.LATERAL_GAIN

        try:
            if self.e_stop == "Warning":
                drive_data.linear.x = 0.0
                drive_data.angular.z = 0.0

            if self.limo_mode == "diff":
                self.drive_pub.publish(drive_data)
            elif self.limo_mode == "ackermann":
                pass

        except Exception as e:
            rospy.logwarn(e)
            
def run():
    new_class = LimoController()
    while not rospy.is_shutdown():
        if new_class.left_receiveimage:
            new_class.roslaunch("marker.launch")

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
