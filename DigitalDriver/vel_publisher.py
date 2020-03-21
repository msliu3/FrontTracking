#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3

if __name__ == "__main__":

    rospy.init_node("vel_cmd_node")
    r = rospy.Rate(10.0)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    speed = 0.0
    omega = 0.0
    twist = Twist(Vector3(speed, 0, 0), Vector3(0, 0, omega))

    while not rospy.is_shutdown():

        pub.publish(twist)

        r.sleep()