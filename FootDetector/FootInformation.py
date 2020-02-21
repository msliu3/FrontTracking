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
2020-2-21
这次修改主要是重新校订，IR image到脚的角度的定义域
"""
import threading

from FootDetector import ProcessFunc as pf
import cv2 as cv
from matplotlib import pyplot as plt

from threading import Thread


class FootInformation(Thread):
    def __init__(self, length=100):
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

        self.list_left_line = []
        self.list_right_line = []
        self.list_left_rect = []
        self.list_right_rect = []
        self.list_length = length

        # 记录两只脚的大小和位置
        self.foot_position_left_x = 0
        self.foot_position_left_y = 0
        self.foot_size_left_w = 0
        self.foot_size_left_h = 0

        # center position
        self.foot_position_right_x = 0
        self.foot_position_right_y = 0
        self.foot_size_right_w = 0
        self.foot_size_right_h = 0

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
            self.foot_position_left_x = rect_left[0][0]
            self.foot_position_left_y = rect_left[0][1]
            self.foot_size_left_w = w0
            self.foot_size_left_h = h0
            self.foot_position_right_x = rect_right[0][0]
            self.foot_position_right_y = rect_right[0][1]
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

        if abs(rect_left[2]) == 90:
            if self.foot_size_left_w > self.foot_size_left_h:
                self.left_rect = 0.0
            else:
                self.left_rect = 90.0
        else:
            self.left_rect = -rect_left[2]

        # 对矩形角度为90的重新定义
        if abs(rect_right[2]) == 90:
            if self.foot_size_right_w > self.foot_size_right_h:
                self.right_rect = 0.0
            else:
                self.right_rect = 90.0
        else:
            self.right_rect = 90 + rect_right[2]

        if self.right_rect < 30:
            self.right_rect = 90.0
        if self.left_rect < 30:
            self.left_rect = 90.0

        # -------------------------------------------------------------------------
        self.left_line = -line_left
        self.right_line = line_right
        if self.right_line < 0:
            temp = 90 + self.right_line
            self.right_line = 90 + temp
        if self.left_line < 0:
            temp = 90 + self.left_line
            self.left_line = 90 + temp

        text = "left: " + str(round(self.left_line, 2)) + " right: " + str(round(self.right_line, 2))
        cv.putText(self.image, text, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
        # print(self.left_line, self.right_line)

    def assign_img_and_pattern(self, np_ir, foot_pattern):
        self.image = np_ir
        self.foot_point_two = foot_pattern
        pass

    def assign_data_to_draw(self):
        self.list_left_line.append(self.left_line)
        self.list_right_line.append(self.right_line)
        self.list_left_rect.append(self.left_rect)
        self.list_right_rect.append(self.right_rect)
        pass

    def draw_pic(self):
        plt.figure(1)
        plt.subplot(511)
        plt.plot(self.mean_filter(self.list_left_line))
        plt.subplot(512)
        plt.plot(self.mean_filter(self.list_right_line))
        plt.subplot(513)
        plt.plot(self.mean_filter(self.list_left_rect))
        plt.subplot(514)
        plt.plot(self.mean_filter(self.list_right_rect))
        plt.subplot(515)
        plt.plot(self.gradient(self.mean_filter(self.list_left_line)))
        plt.show()
        pass

    def mean_filter(self, list, num=3):
        """
        这个函数用于判断，这个人是处于，运动的状态 还是 原地旋转
        :return:
        """
        list.insert(0, list[0])
        for i in range(1, len(list) - num + 1):
            sum = 0
            for j in range(num):
                sum += list[i + j]
            list[i] = sum / num
        return list[1:]
        pass

    def gradient(self, list):
        new_list = []
        for i in range(len(list) - 1):
            new_list.append(list[i + 1] - list[i])
        return new_list

    def clear_current_info(self):
        self.left_rect = 0.0
        self.right_rect = 0.0
        self.left_line = 0.0
        self.right_line = 0.0

        # 记录两只脚的大小和位置
        self.foot_position_left_x = 0
        self.foot_position_left_y = 0
        self.foot_size_left_w = 0
        self.foot_size_left_h = 0

        # center position
        self.foot_position_right_x = 0
        self.foot_position_right_y = 0
        self.foot_size_right_w = 0
        self.foot_size_right_h = 0
        pass


if __name__ == '__main__':
    list1 = [4, 4, 4, 1, 4, 4, 4, 5, 5]
    print(max(list1))
    list1.remove(5)
    print(list1)
    foot = FootInformation()
    list1 = foot.mean_filter(list1, 3)
    print(list1)
    list2 = foot.gradient(list1)
    print(list2)
