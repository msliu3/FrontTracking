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

用过FootInformation 和 LegInformation

"""
import math

import LegDetector.LegInformation as LegLidar
import FootDetector.FootInformation as FootInformation
import threading


class MatchCase(object):
    def __init__(self, foot, base_size=30):
        # 打开ros，开始读取leg information
        self.leg = LegLidar.LegInformation()
        thread_start = threading.Thread(target=self.leg.loop, args=())
        thread_start.start()
        # 因为footinformation不是已独立存在的类，所以这里选用传入的方式
        # self.foot = FootInformation.FootInformation()
        self.foot = foot
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

        # 判断那只脚在前那只脚在后
        self.front = ""
        self.not_distinguish = False
        self.no_result = False
        self.step_distance = 0.0
        self.distance_flag = ""
        self.distance_same = False

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

        self.average_l = []
        self.average_r = []
        self.average_size = 15
        self.a_l = 0.0
        self.a_r = 0.0
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
        dis = (self.leg.left_leg_x + self.leg.right_leg_x) / 2
        if self.not_distinguish:
            if -0.15 < dis < 0.05:
                self.rotate = True
                # print(self.foot.left_line, self.foot.right_line)
                print("rotate")
                self.base_foot()
                self.foot.assign_data_to_draw()
            else:
                if dis < -0.15:
                    self.back = True
                    print("back")
                    self.go_back_or_forward(dis)
                    return self.expect_x, self.expect_theta
                elif dis > 0.05:
                    print("forward")
                    self.forward = True
                    self.go_back_or_forward(dis)
                    return self.expect_x, self.expect_theta
        else:
            if self.front != "":
                # print("dis", dis, abs(self.leg.left_leg_x - self.leg.right_leg_x))
                print("turing and forward")
                self.turning = True
                self.go_turning_and_forward()
                # print(self.leg.left_leg_x, self.leg.right_leg_x)
                return self.expect_x, self.expect_theta
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
        if self.front == "left":
            self.expect_theta = 90 - self.a_l
            self.expect_x = self.leg.left_leg_x
        else:
            self.expect_theta = -(90 - self.a_r)
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
        self.average_foot()
        self.no_result = False
        dis = self.distance_detect_front_foot()
        self.img_detect_front_foot()
        if self.distance_same:
            self.not_distinguish = True
            self.front = ""
            # print("same ", dis)
        else:
            if self.distance_flag == self.img_flag:
                self.front = self.distance_flag
                self.not_distinguish = False
                # print("front is", self.front, dis)
            else:
                self.clear_all()
                self.no_result = True
        if self.not_distinguish and dis == 0:
            self.clear_all()
            self.no_result = True
        # print("front is ", self.front, "is same?", self.not_distinguish, "has result?", self.no_result, dis)
        pass

    def img_detect_front_foot(self):
        """
        2.从图像上讲，近大远小哪只脚的面积比较大
                            起始点比较考前
        :return:
        """
        area_left = self.foot.foot_size_left_w * self.foot.foot_size_left_h
        area_right = self.foot.foot_size_right_w * self.foot.foot_size_right_h
        if area_right != 0:
            result = area_left / area_right
        else:
            result = 100
        if result > 1:
            #     左脚比右脚大
            temp_result_area = "left"
        else:
            temp_result_area = "right"
        if 0.5 < result < 1.5:
            self.img_size_same = True
        else:
            self.img_size_same = False

        # original position
        if self.foot.foot_position_left_y > self.foot.foot_position_right_y:
            temp_result_distance = "left"
        else:
            temp_result_distance = "right"

        # 这里面没有判断位置近似的情况
        if temp_result_distance == temp_result_area:
            self.img_flag = temp_result_area
            self.img_dis_same = False
        else:
            # print("can not distinguish!")
            self.img_dis_same = True
        # print("img: ", temp_result_area, temp_result_distance, "is same ", self.img_size_same,self.img_dis_same)
        return result

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

    def average_foot(self):
        if len(self.average_l) < self.average_size:
            self.average_l.append(self.foot.left_line)
            self.average_r.append(self.foot.right_line)
        else:
            self.average_l.pop(0)
            self.average_r.pop(0)
            self.average_l.append(self.foot.left_line)
            self.average_r.append(self.foot.right_line)
            sum_l = 0
            for i in self.average_l:
                sum_l += i
            avg_l = sum_l / self.average_size
            sum_r = 0
            for i in self.average_r:
                sum_r += i
            avg_r = sum_r / self.average_size
            self.a_l = avg_l
            self.a_r = avg_r
            return avg_l, avg_r

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
