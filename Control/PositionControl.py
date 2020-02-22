#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   PositionControl.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/29 18:43   msliu      1.0      

@Description
------------
机器人油门输入：
    1.线速度
    2.角速度
    3.旋转半径
控制输出结果：
    以机器人为原点[0, 0, 0] -> 期望位置[x, y, theta]
    12-30:因为我们的机器人不能平移，所以不设y值
    其中该坐标系遵循右手系：

    theta往左为正，往右为负

    -------------------------------------------------
    12-30：
    分别设置几个情况，进行匹配
    1.原地旋转
    2.前进转弯（小角度和大角度）
    3.直线前进
    4.后退

"""
import math
import time


class PositionControl(object):
    def __init__(self, radius=56):
        # 期望的位置
        self.expect_x = 0.0
        # 因为我们的车不能平移所以不给y值
        # self.expect_y = 0.0
        # 这里还没定义好是使用 角度 还是 弧度
        self.expect_theta = 0.0
        self.robot_r = radius

        # 需要输出的油门信息 speed m/s rad/s(弧度每秒) 旋转半径r cm
        self.speed = 0.0
        self.omega = 0.0
        self.radius = 0.0
        self.running_time = 0.0

        self.action_over = True
        pass

    def clear_driver(self):
        self.speed = 0
        self.omega = 0
        self.radius = 0
        pass

    def design_path_rotate(self):
        if self.expect_theta == 0:
            pass
        else:
            if self.expect_theta > 0:
                self.omega = 0.4
            else:
                self.omega = -0.4
            rad = math.radians(self.expect_theta)
            self.running_time = float(abs(rad) / abs(self.omega))
        pass

    def design_path_forward_and_turning(self):
        """
        直行和行走转弯都可以处理
        [0, 5]    (15, 30]    (30, 45]    (45, 90]
        -------------------------------------------
        正无穷      r ->2r        r           r/2
        :return:
        """
        # 赋予速度定值
        if self.expect_theta > 0:
            self.omega = 0.2
        else:
            self.omega = -0.2

        abs_theta = abs(self.expect_theta)
        if abs_theta <= 25:
            # 判定为直行
            self.omega = 0
            self.design_path_forward_and_back()
            return

        elif 25 < abs_theta <= 55:
            self.radius = self.robot_r * 2 - ((abs_theta - 15) / (55 - 15) * self.robot_r)
        elif 85 < abs_theta <= 90:
            self.radius = self.robot_r / 2
        print(self.expect_x, self.expect_theta)
        rad, degree = self.calculate_degree(self.radius)
        # print("rad: ", rad, "degree: ", degree)
        self.running_time = float(abs(rad / self.omega))

        # print(self.running_time)
        pass

    def calculate_degree(self, l):
        if self.expect_theta!=90:
            sin = math.sin(self.expect_x / (l - self.robot_r / 4) * 100)
        else:
            sin = math.sin(self.expect_x / (l + self.robot_r / 4) * 100)
        rad = math.asin(sin)
        return rad, math.degrees(rad)

    def design_path_forward_and_back(self):
        # 赋予直行速度定值
        if self.expect_x < 0:
            self.speed = -0.2
        elif self.expect_x > 0:
            self.speed = 0.2

        else:
            self.clear_driver()
            return
        self.running_time = float(abs(self.expect_x) / abs(self.speed))

    def set_expect(self, expect_x, expect_theta):
        self.expect_x = expect_x
        self.expect_theta = expect_theta
        pass

    def set_driver(self, control_driver):
        control_driver.speed = self.speed
        control_driver.radius = self.radius
        control_driver.omega = self.omega
        pass

    def action_forward_back(self, control_driver):
        self.action_over = False
        self.design_path_forward_and_back()
        print("action: speed", self.speed, " omega", self.omega, " radius", self.radius, " time", self.running_time)
        if self.speed != 0 or self.omega != 0:
            self.set_driver(control_driver)
            time.sleep(self.running_time)
        self.clear_driver()
        self.set_driver(control_driver)
        self.action_over = True

    def action_rotation(self, control_driver):
        self.action_over = False
        self.design_path_rotate()
        # print("action: speed", self.speed, " omega", self.omega, " radius", self.radius, " time", self.running_time)
        if self.speed != 0 or self.omega != 0:
            self.set_driver(control_driver)
            time.sleep(self.running_time)
        self.clear_driver()
        self.set_driver(control_driver)
        self.action_over = True

    def action_forward_and_turning(self, control_driver):
        self.action_over = False
        self.design_path_forward_and_turning()
        # print("action: speed", self.speed, " omega", self.omega, " radius", self.radius, " time", self.running_time)
        if self.speed != 0 or self.omega != 0:
            self.set_driver(control_driver)
            time.sleep(self.running_time)
        self.clear_driver()
        self.set_driver(control_driver)
        self.action_over = True


if __name__ == '__main__':
    pc = PositionControl()
    pc.expect_x = 30
    pc.expect_theta = 60
    pc.design_path_forward_and_back()
