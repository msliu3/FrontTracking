#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   MatchCase.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/30 17:52   msliu      1.0      

@Description
------------
这个程序用来匹配几种情况
1.原地旋转
2.前进转弯（小角度和大角度）
3.直线前进
4.后退

2.0 version

用过FootInformation 和 LegInformation

"""
import math

import LegDetector.LegInformation as LegLidar
import FootDetector.FootInformation as FootInformation
import threading
from matplotlib import pyplot as plt


class MatchCase(object):
    def __init__(self, foot, base_size=30):
        # 打开ros，开始读取leg information
        self.leg = LegLidar.LegInformation()
        thread_start = threading.Thread(target=self.leg.loop, args=())
        thread_start.start()
        # 因为footinformation不是已独立存在的类，所以这里选用传入的方式
        # self.foot = FootInformation.FootInformation()
        self.foot = FootInformation.FootInformation()
        """
        下面是我需要得到的东西，期望的x和theta
        """
        self.expect_x = 0.0
        self.expect_theta = 0.0

        # 这是四种case的情况
        self.rotate = False
        self.forward = False
        self.turning = False
        self.back = False

        self.state = None

        # 判断那只脚在前那只脚在后
        self.front = ""
        self.not_distinguish = False
        self.no_result = False
        self.step_distance = 0.0
        self.distance_flag = ""
        self.distance_same = False
        self.dis_in_place = False

        self.img_flag = ""
        self.img_size_same = False
        self.img_dis_same = False

        # base line foot angle
        self.base_foot_l = []
        self.base_foot_r = []
        self.base_l = ()
        self.base_r = ()
        self.base_size = base_size
        self.base_cal_flag = False

        pass

    def detect_case(self):
        """
        判断人是不是在原地
        以两个轮子连线的中点为基准线
        向前5cm
        向后15cm
        均认为是原地动作
        :return:
        """
        self.clear_case()
        # print("foot information", self.foot.left_line, self.foot.right_line)
        # print("Leg Information :",self.leg.left_leg_x,self.leg.right_leg_x)
        if self.front == "left":
            front_x = self.leg.left_leg_x
        else:
            front_x = self.leg.right_leg_x

        if self.front != "":

            # print(front_x)
            if -0.15 < front_x < 0.05:
                self.rotate = True
                print("rotate_front")
                self.state = "rotate_front"
                # self.base_foot()
            elif front_x < -0.15:
                self.back = True
                print("back")
                self.state = "back"
                self.go_back_or_forward(front_x)
                return self.expect_x, self.expect_theta
            elif front_x > 0.05:
                if self.foot.left_line != 0:
                    print("turing and forward:", self.front)
                    self.turning = True
                    self.go_turning_and_forward()
                    # print("---------------------------------------------")
                    # print(self.expect_theta)
                    # print("---------------------------------------------")
                    return self.expect_x, self.expect_theta
                else:
                    self.forward = True
                    self.go_back_or_forward(front_x)
                    print("forward")
                    self.state = "forward"
                    return self.expect_x, self.expect_theta
        # 总是能分出前后脚的所以不需要判断两脚并一块的情况
        # else:
        # print("No result:not_dis:",self.not_distinguish," front:",self.front)
        return 0, 0

    def go_back_or_forward(self, dis):
        self.expect_x = dis
        self.expect_theta = 0

    def go_rotate(self):
        """
        1.moved
        2.has a big angle
        :return:
        """
        # if
        pass

    def go_turning_and_forward(self):
        # print(self.a_l, self.a_r)
        # 判断一下这只脚的情况：前行左转右转
        if self.front == "left":
            if self.foot.left_line > 100:
                self.expect_theta = -90.0
                print("turn right:left")
                self.state = "turn right:left"
            elif self.foot.left_rect == 90:
                self.expect_theta = 0
                self.turning = False
                self.forward = True
                print("forward:l")
                self.state = "forward:l"
            else:
                self.expect_theta = 90 - self.foot.left_line
                print("turn left")
                self.state = "turn left"
            self.expect_x = self.leg.left_leg_x
        else:
            if self.foot.right_line > 100:
                self.expect_theta = 90
                print("turn left:right")
                self.state = "turn left:right"
            elif self.foot.right_rect == 90:
                self.expect_theta = 0
                print("forward:r")
                self.turning = False
                self.forward = True
                self.state = "forward:r"
            else:
                self.expect_theta = -(90 - self.foot.right_line)
                print("turn right")
                self.state = "turn right"
            self.expect_x = self.leg.right_leg_x
        pass

    def detect_front_and_back_foot(self):
        """
        已有经验
        1.从距离上讲，那只脚的x比较大可以认为在前
        2.从图像上讲，近大远小哪只脚的面积比较大
                            起始点比较考前
        :return:
        """
        self.clear_expect()
        self.no_result = False
        dis = self.distance_detect_front_foot()
        if self.distance_same:
            self.not_distinguish = True
            self.front = self.distance_flag
        else:
            self.front = self.distance_flag
            self.not_distinguish = False
        # print("front is ", self.front, "is same?", self.not_distinguish, "has result?", self.no_result, dis)
        pass

    def distance_detect_front_foot(self, step_dis=0.075):
        """
        1.从距离上讲，那只脚的x比较大可以认为在前
        :return:
        """
        if self.leg.left_leg_x > self.leg.right_leg_x:
            self.distance_flag = "left"
        else:
            self.distance_flag = "right"
        if abs(self.leg.left_leg_x - self.leg.right_leg_x) < step_dis:

            self.distance_same = True
        else:
            self.distance_same = False

        # print("distance: the front foot is " + self.distance_flag, "is same: ", self.distance_same, " ",
        #       abs(self.leg.left_leg_x - self.leg.right_leg_x))
        self.step_distance = abs(self.leg.left_leg_x - self.leg.right_leg_x)
        return self.step_distance

    def base_foot(self):
        if len(self.base_foot_l) < self.base_size:
            self.base_foot_l.append(self.foot.left_line)
            self.base_foot_r.append(self.foot.right_line)
        else:
            if not self.base_cal_flag:
                self.base_foot_l.remove(max(self.base_foot_l))
                self.base_foot_l.remove(min(self.base_foot_l))
                self.base_l = (min(self.base_foot_l), max(self.base_foot_l))

                self.base_foot_r.remove(max(self.base_foot_r))
                self.base_foot_r.remove(min(self.base_foot_r))
                self.base_r = (min(self.base_foot_r), max(self.base_foot_r))
                print("base_line:", self.base_l, self.base_r)
                self.base_cal_flag = True
            else:
                pass

    def clear_all(self):
        self.expect_x = 0.0
        self.expect_theta = 0.0

        # 这是四种case的情况
        self.rotate = False
        self.forward = False
        self.turning = False
        self.back = False

        # 判断那只脚在前那只脚在后
        self.front = ""
        self.back = ""
        self.not_distinguish = False
        self.no_result = False
        self.step_distance = 0.0
        self.distance_flag = ""
        self.distance_same = False

        self.img_flag = ""
        self.img_size_same = False
        self.img_dis_same = False

    def clear_expect(self):
        self.expect_x = 0
        self.expect_theta = 0

    def clear_case(self):
        # 这是四种case的情况
        self.rotate = False
        self.forward = False
        self.turning = False
        self.back = False

    def clear_foot_and_leg(self):
        # self.foot.clear_current_info()
        self.leg.clear_leg()
