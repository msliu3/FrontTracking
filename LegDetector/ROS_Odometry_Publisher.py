#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
@File    :   DemonstrationProcess.py
@Contact :   liumingshanneo@163.com

@Modify    Time      @Author    @Version    @Desciption
--------   ----      -------    --------    -----------
2020/1/12  21:56       msliu      1.0
"""

import rospy
import roslib
from nav_msgs.msg import Odometry
import tf

class OdometryPublisher(object):
    def __init__(self):
        rospy.init_node('odometry_publisher_node', anonymous=True)
        self.pub = rospy.Publisher('odometry_publisher', Odometry, queue_size=50)
        # self.tf = tf.TransformListener()
        print(rospy.Time.now())
    pass

if __name__ == '__main__':
    op = OdometryPublisher()

