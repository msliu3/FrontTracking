#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from DigitalDriver import ControlDriver as CD
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


if __name__ == "__main__":
    # odom = Odometry()
    # print('X:', odom.X, 'm;   Y:', odom.Y, 'm;   THETA:', odom.THETA / math.pi * 180, '°;')

    cd = CD()

    rospy.init_node('odometry_broadcaster')

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_br = tf.TransformBroadcaster()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(1.0)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, cd.position[2])

        # First, we will broadcast the transform from frame "odom" to the frame "base_link" over tf
        odom_br.sendTransform(
            (cd.position[1], -cd.position[0], 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vtheta))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()