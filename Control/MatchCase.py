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
import LegDetector.LegInformation as LegLidar
import FootDetector.FootInformation as FootInformation
import threading


class MatchCase(object):
    def __init__(self, FootInformation):
        # 打开ros，开始读取leg information
        self.leg = LegLidar.LegInformation()
        thread_start = threading.Thread(target=self.leg.loop, args=())
        thread_start.start()
        # 因为footinformation不是已独立存在的类，所以这里选用传入的方式
        self.foot = FootInformation
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
        if self.not_distinguish:
            dis = (self.leg.left_leg_x + self.leg.right_leg_x) / 2
            print("dis", dis)
            if -0.15 < dis < 0.05:
                self.rotate = True
                print("rotate")
            else:
                if dis < -0.15:
                    self.back = True
                    print("back")
                    self.go_back(dis)
                    if self.expect_x != 0:
                        return self.expect_x, self.expect_theta
                elif dis > 0.05:
                    print("forward")
                    self.forward = True
        else:
            if self.front != "":
                print("turing and forward")
                self.turning = True
        return 0, 0

    def go_back(self, dis):
        self.expect_x = dis
        self.expect_theta = 0

    def detect_front_and_back_foot(self):
        """
        已有经验
        1.从距离上讲，那只脚的x比较大可以认为在前
        2.从图像上讲，近大远小哪只脚的面积比较大
                            起始点比较考前
        :return:
        """
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
                self.clear()
                self.no_result = True
        if self.not_distinguish and dis == 0:
            self.clear()
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

    def distance_detect_front_foot(self):
        """
        1.从距离上讲，那只脚的x比较大可以认为在前
        :return:
        """
        if self.leg.left_leg_x > self.leg.right_leg_x:
            self.distance_flag = "left"
        else:
            self.distance_flag = "right"
        if abs(self.leg.left_leg_x - self.leg.right_leg_x) < 0.05:

            self.distance_same = True
        else:
            self.distance_same = False

        # print("distance: the front foot is " + self.distance_flag, "is same: ", self.distance_same, " ",
        #       abs(self.leg.left_leg_x - self.leg.right_leg_x))
        self.step_distance = abs(self.leg.left_leg_x - self.leg.right_leg_x)
        return self.step_distance

    def clear(self):
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
