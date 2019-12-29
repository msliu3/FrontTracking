#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   FootInformation.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/5 22:35   msliu      1.0      

@Description
------------
None
"""
import threading

import numpy as np
import ProcessFunc as pf
import cv2 as cv
from matplotlib import pyplot as plt
import math

import DemonstrationProcess
from threading import Thread


class FootInformation(Thread):
    def __init__(self):
        """

        :param np_ir:  source image
        :param foot_pattern:
                        The length of list is two
                        numpy points
        """
        self.image = None
        self.foot_point_two = None

        # 所有的角度定义的都是，和竖直方向的夹角，即现实中脚与前进方向的成角
        self.left_rect = 0.0
        self.right_rect = 0.0
        self.left_line = 0.0
        self.right_line = 0.0

        self.list_left = []
        self.list_right = []
        self.human = []

        # 记录两只脚的大小和位置
        self.foot_position_left_x = 0
        self.foot_position_left_y = 0
        self.foot_size_left_w = 0
        self.foot_size_left_h = 0

        self.foot_position_right_x = 0
        self.foot_position_right_y = 0
        self.foot_size_right_w = 0
        self.foot_size_right_h = 0

        # 做控制的变量
        self.base_line = 0.0
        pass

    def draw_and_obtain_element(self, np_ir, foot_pattern):
        self.assign_img_and_pattern(np_ir, foot_pattern)
        item0 = self.foot_point_two[0]
        item1 = self.foot_point_two[1]
        # 先提取一下正常长方形的X值，用来判断左右脚
        x0, y0, w0, h0 = pf.draw_normal_rectangle(self.image, item0, color=(0, 0, 255))
        x1, y1, w1, h1 = pf.draw_normal_rectangle(self.image, item1, color=(0, 0, 255))
        if x0 < x1:
            # item0为左脚，item1为右脚
            rect_left = pf.draw_min_rectangle(self.image, item0)
            rect_right = pf.draw_min_rectangle(self.image, item1)
            line_left = pf.draw_min_line(self.image, item0)
            line_right = pf.draw_min_line(self.image, item1)
            self.foot_position_left_x = x0
            self.foot_position_left_y = y0
            self.foot_size_left_w = w0
            self.foot_size_left_h = h0
            self.foot_position_right_x = x1
            self.foot_position_right_y = y1
            self.foot_size_right_w = w1
            self.foot_size_right_h = h1
        else:
            # item1为左脚，item0为右脚
            rect_left = pf.draw_min_rectangle(self.image, item1)
            rect_right = pf.draw_min_rectangle(self.image, item0)
            line_left = pf.draw_min_line(self.image, item1)
            line_right = pf.draw_min_line(self.image, item0)
            self.foot_position_left_x = x1
            self.foot_position_left_y = y1
            self.foot_size_left_w = w1
            self.foot_size_left_h = h1
            self.foot_position_right_x = x0
            self.foot_position_right_y = y0
            self.foot_size_right_w = w0
            self.foot_size_right_h = h0

        self.left_rect = -rect_left[2]
        self.right_rect = 90 + rect_right[2]
        self.left_line = -line_left
        self.right_line = line_right

        text = "left: " + str(round(self.left_rect, 2)) + " right: " + str(round(self.right_rect, 2))
        cv.putText(self.image, text, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        # print(self.left_line, self.right_line)

    def assign_img_and_pattern(self, np_ir, foot_pattern):
        self.image = np_ir
        self.foot_point_two = foot_pattern
        pass

    def assign_data_to_draw(self):
        self.list_left.append(abs(self.left_line))
        self.list_right.append(abs(self.right_line))
        self.human.append(abs(self.left_line) - abs(self.right_line))
        pass

    def draw_pic(self):
        plt.figure(1)
        plt.subplot(311)
        plt.plot(self.list_left)
        plt.subplot(312)
        plt.plot(self.list_right)
        plt.subplot(313)
        plt.plot(self.human)
        plt.show()
        pass

    def is_moving(self):
        """
        这个函数用于判断，这个人是处于，运动的状态 还是 原地旋转
        :return:
        """
        pass

    def Rotate_in_place(self, kp=1):
        expect_angle = abs(self.left_line) - abs(self.right_line)
        return (self.base_line - expect_angle) * kp

    def loop(self):
        while True:
            self.Rotate_in_place()

if __name__ == '__main__':
    thread1 = threading.Thread(target=DemonstrationProcess.start_Demon(), args=())
    thread1.start()



