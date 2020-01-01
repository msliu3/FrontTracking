#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   LegInformation.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/11/28 11:49   msliu      1.0

@Description
------------
subscribed neo_remarker
obtain leg information
对接ROS的文件：
    监听两个话题：/Pose2D 相当于获得车的历程信息
                  /marker 获得腿部信息

    将两个值相减，来获得人腿的绝对坐标

"""

import rospy
from threading import Thread
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D


def singleton(cls, *args, **kw):
    instances = {}

    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]

    return _singleton


@singleton
class LegInformation(Thread):
    def __init__(self, length=10):
        Thread.__init__(self)
        self.left_leg_x = 0.0
        self.left_leg_y = 0.0
        self.right_leg_x = 0.0
        self.right_leg_y = 0.0
        self.height = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.prev_data_x = []
        self.prev_data_y = []
        self.prev_data_length = length
        rospy.init_node('neo_get_leg_node', anonymous=True)
        print("has been started LegInformation")
        pass

    def listener_neo_marker(self):
        rospy.Subscriber("/neo_marker", Marker, self.neo_marker_callback)
        pass

    def neo_marker_callback(self, data):
        """
        obtain leg information
        """
        temp_x = data.pose.position.x - self.robot_x
        temp_y = data.pose.position.y - self.robot_y

        # 这里需要修改，这里是默认了左腿的y值大于右腿的y值
        if temp_y > 0:
            self.left_leg_x = temp_x
            self.left_leg_y = temp_y
            # print("left: " + str(self.left_leg_x) + " " + str(self.left_leg_y))
        else:
            self.right_leg_x = temp_x
            self.right_leg_y = temp_y
            # print("right: " + str(self.right_leg_x) + " " + str(self.right_leg_y))
        pass

    def listener_pose_2D(self):
        rospy.Subscriber("/pose2D", Pose2D, self.pose2D_callback)
        pass

    def pose2D_callback(self, data):
        self.robot_x = data.x
        self.robot_y = data.y
        pass

    def loop(self):
        leg = LegInformation()
        leg.listener_pose_2D()
        leg.listener_neo_marker()
        rospy.spin()
        pass


if __name__ == '__main__':
    leg = LegInformation()
    leg.listener_pose_2D()
    leg.listener_neo_marker()
    rospy.spin()
