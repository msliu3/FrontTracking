#!/usr/bin/env python
# -*- coding:utf-8 -*-

import math
import time
import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def euler_to_quaternion(roll, pitch, yaw):
    w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [w, x, y, z]


def quaternion_to_euler(w, x, y, z):
    roll = math.atan2(2*(w*x+y*z), 1-2*(x**2+y**2))
    yaw = math.atan2(2*(w*z + x*y), 1-2*(z**2+y**2))
    pitch = math.asin(2*(w*y-x*z))
    return [roll, pitch, yaw]


def callback(imu):
    msg = quaternion_to_euler(imu.orientation.w,
                              imu.orientation.x,
                              imu.orientation.y,
                              imu.orientation.z)
    roll = msg[0]/math.pi*180
    pitch = msg[1]/math.pi*180
    yaw = msg[2]/math.pi*180
    print("Imu echo: roll = %.3f,  pitch = %.3f,  yaw = %.3f" % (roll, pitch, yaw))


def raw_callback(imu):
    msg = quaternion_to_euler(imu.orientation.w,
                              imu.orientation.x,
                              imu.orientation.y,
                              imu.orientation.z)
    print("Raw Imu echo: yaw = %.3f\n\n" % (msg[2]/math.pi*180))
    # time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("imu_echo_node")
    r = rospy.Rate(10)
    # sub = rospy.Subscriber("imu/data", Imu, callback)
    sub_raw = rospy.Subscriber("imu/data_raw", Imu, raw_callback)

    rospy.spin()
