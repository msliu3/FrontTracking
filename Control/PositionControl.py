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
    其中该坐标系遵循右手系：

    theta往左为正，往右为负
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
        pass

    def clear_driver(self):
        self.speed = 0
        self.omega = 0
        self.radius = 0
        pass

    def design_path(self):
        """
        [0, 15]    (15, 30]    (30, 45]    (45, 90]
        -------------------------------------------
        正无穷      r ->2r        r           r/2
        :return:
        """
        if self.expect_theta > 0:
            self.omega = 0.3
        else:
            self.omega = -0.3

        abs_theta = abs(self.expect_theta)
        if abs_theta <= 15:
            self.omega = 0
        elif 15 < abs_theta <= 30:
            self.radius = self.robot_r * 2 - ((abs_theta - 15) / (30 - 15) * self.robot_r)
            print()
        elif 30 < abs_theta <= 45:
            self.radius = self.robot_r
        elif 45 < abs_theta <= 90:
            self.radius = self.robot_r / 2
        rad, degree = self.calculate_degree(self.radius)
        print("rad: ", rad, "degree: ", degree)
        self.running_time = abs(rad / self.omega)
        print(self.running_time)
        pass

    def calculate_degree(self, l):
        sin = math.sin(self.expect_x / l)
        rad = math.asin(sin)
        return rad, math.degrees(rad)

    def set_expect(self, expect_x, expect_theta):
        self.expect_x = expect_x
        self.expect_theta = expect_theta
        pass

    def set_driver(self, control_driver):
        control_driver.radius = self.radius
        control_driver.omega = self.omega
        pass

    def action(self, control_driver):
        self.design_path()
        self.set_driver(control_driver)
        print("into", self.omega)
        time.sleep(self.running_time)
        self.clear_driver()
        self.set_driver(control_driver)


if __name__ == '__main__':
    pc = PositionControl()
    pc.expect_x = 30
    pc.expect_theta = 60
    pc.design_path()