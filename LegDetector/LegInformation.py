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
import threading

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
from threading import Thread
from sklearn.cluster import KMeans
import numpy as np
import time
import math


def singleton(cls, *args, **kw):
    instances = {}

    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]

    return _singleton


@singleton
class LegInformation(Thread):
    def __init__(self, length=50):
        Thread.__init__(self)
        self.left_leg_x = 0.0
        self.left_leg_y = 0.0
        self.right_leg_x = 0.0
        self.right_leg_y = 0.0
        self.height = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.theta = 0.0

        self.prev_data = []
        self.prev_data_y = []
        self.prev_data_length = length
        rospy.init_node('neo_get_leg_node', anonymous=True)
        self.prev_leg = [
            -0.13231158,
            0.09736817,
            -0.12490384,
            0.09615281,
            -0.12326501,
            0.08969253,
            -0.13799247,
            0.10617598,
            -0.11245747,
            0.14171072,
            -0.09778179,
            -0.15025203,
            0.16142259,
            -0.08951196,
            0.17213701,
            -0.08138719,
            0.16241966,
            -0.08664881,
            0.15453492,
            -0.08944231,
            0.15214336,
            -0.09137838,
            0.11405817,
            -0.09202546,
            0.13354793,
            -0.09606664,
            0.14175811,
            -0.09467949,
            0.15366877,
            -0.09725611,
        ]

        print("It has been started LegInformation")
        pass

    def listener_neo_marker(self):
        """
        read topic called "/neo_marker"
        when some data is read, it will execute neo_marker_callback
        :return:
        """
        rospy.Subscriber("/neo_marker", Marker, self.neo_marker_callback)
        pass

    def neo_marker_callback(self, data):
        """
        obtain leg information
        """
        # print("left: " + str(self.left_leg_x) + " " + str(self.left_leg_y))

        original_x = data.pose.position.x - self.robot_x
        original_y = data.pose.position.y - self.robot_y
        theta = self.theta

        # 坐标系变换
        trans = np.array([[math.cos(theta), math.sin(theta)],
                          [-math.sin(theta), math.cos(theta)]])
        result = np.dot(trans, np.array([[original_x], [original_y]]))
        temp_x = result[0]
        temp_y = result[1]

        # k-means 判断左右腿
        self.kmeans_detect_left_and_right(temp_x, temp_y)

        pass

    def listener_pose_2D(self):
        rospy.Subscriber("/pose2D", Pose2D, self.pose2D_callback)
        pass

    def pose2D_callback(self, data):
        self.robot_x = data.x
        self.robot_y = data.y
        self.theta = data.theta
        pass

    def kmeans_detect_left_and_right(self, temp_x, temp_y):
        foot = np.array([temp_x, temp_y])
        # print(foot)
        self.prev_data.append(temp_y)
        if len(self.prev_data) > self.prev_data_length:
            self.prev_data.pop(0)
        # self.prev_data.pop(-1)
        # temp_list = self.prev_data
        # self.prev_data.append(temp_y)
        # temp_list.extend(self.prev_leg)
        # temp_list.append(temp_y)
        temp_np = np.array(self.prev_data).reshape(-1, 1)
        if len(self.prev_data) == 1:
            return
        # print(temp_np)
        result = KMeans(n_clusters=2).fit_predict(temp_np)
        sum_flag0 = 0
        sum_flag1 = 0
        for i in range(len(result)):
            if result[i] == 0:
                sum_flag0 += temp_np[i]
            else:
                sum_flag1 += temp_np[i]
        if sum_flag0 > sum_flag1:
            if result[-1] == 0:
                self.left_leg_x = temp_x
                self.left_leg_y = temp_y
                # print("left: " + str(self.left_leg_x) + " " + str(self.left_leg_y))
            else:
                self.right_leg_x = temp_x
                self.right_leg_y = temp_y
                # print("right: " + str(self.right_leg_x) + " " + str(self.right_leg_y))
        else:
            if result[-1] == 1:
                self.left_leg_x = temp_x
                self.left_leg_y = temp_y
                # print("left: " + str(self.left_leg_x) + " " + str(self.left_leg_y))
            else:
                self.right_leg_x = temp_x
                self.right_leg_y = temp_y
                # print("right: " + str(self.right_leg_x) + " " + str(self.right_leg_y))
        # print(result)
        # print(self.left_leg_x, self.right_leg_x)

    def clear_leg(self):
        self.left_leg_x = 0
        self.left_leg_y = 0
        self.right_leg_x = 0
        self.right_leg_y = 0
        pass

    def detect_has_a_human(self):
        lx = self.left_leg_x
        ly = self.left_leg_y
        rx = self.right_leg_x
        ry = self.right_leg_y
        while True:
            time.sleep(2)
            if lx == self.left_leg_x or ly == self.left_leg_y or rx == self.right_leg_x or ry == self.right_leg_y:
                self.clear_leg()
            else:
                lx = self.left_leg_x
                ly = self.left_leg_y
                rx = self.right_leg_x
                ry = self.right_leg_y

    def loop(self):
        leg = LegInformation()
        leg.listener_pose_2D()
        leg.listener_neo_marker()
        # It could be waste resource
        # thread_start = threading.Thread(target=leg.detect_has_a_human, args=())
        # thread_start.start()
        rospy.spin()
        pass


if __name__ == '__main__':
    leg = LegInformation()
    leg.listener_pose_2D()
    leg.listener_neo_marker()
    # It could be waste resource
    # thread_start = threading.Thread(target=leg.detect_has_a_human, args=())
    # thread_start.start()
    rospy.spin()
