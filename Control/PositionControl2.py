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


class PositionControl2(object):
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
        pass

    # 由外部给入预测信号
    def set_expect(self, expect_x, expect_theta):
        self.expect_x = expect_x
        self.expect_theta = expect_theta
        pass

    # 顶层判断，决策运动状态
    def top_decision(self, control_driver):
        """
        直行和行走转弯都可以处理
        [0, 5]    (15, 30]    (30, 45]    (45, 90]
        -------------------------------------------
        正无穷      r ->2r        r           r/2
        :return:
        """
        abs_theta = abs(self.expect_theta)
        # 停止状态
        if abs_theta == 0 and self.expect_x == 0:
            self.clear_driver(0.2, 20000, control_driver)
            print("Action mode: Stop!!!\n")
            return
        # 直行状态
        elif abs_theta <= 25:
            self.action_forward_back(control_driver)
            return
        # 前进转弯
        elif abs_theta <= 55 or 85 < abs_theta <= 90:
            self.action_forward_and_turning(control_driver)
            return
        else:
            return

    # 计算转弯半径
    def design_radius(self):
        abs_theta = abs(self.expect_theta)
        if abs_theta <= 55:
            self.radius = self.robot_r * 2 - ((abs_theta - 15) / (55 - 15) * self.robot_r)
        elif 85 < abs_theta <= 90:
            self.radius = self.robot_r / 2

    # 传输信号
    def set_driver(self, control_driver, times):
        control_driver.speed = self.speed
        control_driver.radius = self.radius
        control_driver.omega = self.omega
        time.sleep(times)
        return

    # 起步动作, 用插值，总用时start_time, 分step步, 最终speed为value
    def start_driver(self, mode, start_time, step, value, control_driver):
        time_step = start_time / step
        # 转弯的插值
        if mode == "Turn":
            self.design_radius()
            omega_step = value / step
            for i in range(step):
                self.omega += omega_step
                self.set_driver(control_driver, start_time / step)
        # 直行的插值
        elif mode == "Straight":
            speed_step = value / step
            for i in range(step):
                self.speed += speed_step
                self.set_driver(control_driver, start_time / step)
        else:
            return

    # 刹车动作, 用插值，总用时stop_time, 分step步
    def clear_driver(self, stop_time, step, control_driver):
        time_step = stop_time / step
        # 转弯的插值
        if self.omega != 0:
            omega_step = self.omega / step
            for i in range(step):
                self.omega = self.omega - omega_step
                self.set_driver(control_driver, stop_time / step)
            self.radius = 0
            self.omega = 0
        # 直行的插值
        elif self.speed != 0:
            speed_step = self.speed / step
            for i in range(step):
                self.speed = self.speed - speed_step
                self.set_driver(control_driver, stop_time / step)
            self.speed = 0
        else:
            time.sleep(0.2)
            return

    # 插值，主要用于直行的前进和后退改变，直行到转弯的改变，转弯到转弯的改变，
    # 最终speed或者omega变为value值
    def interpolation(self, mode, times, steps, value, control_driver):
        if mode == "Straight":
            value_step = (value - self.speed) / steps
            time_step = times / steps
            for i in range(steps):
                self.speed += value_step
                self.set_driver(control_driver, time_step)
        elif mode == "Straight2Turn":
            # # 先停止直行
            # self.clear_driver(times / 2, steps, control_driver)
            # self.speed = 0  # 确保无速度信息
            # # 然后开始转弯
            # self.start_driver("Turn", times / 2, steps, value, control_driver)

            # 另外一种方案，直接同时递减speed递增omega
            speed_step = self.speed / steps
            omega_step = value / steps
            for i in range(steps):
                self.speed -= speed_step
                self.omega += omega_step
                self.set_driver(control_driver, times / steps)
            self.speed = 0  # 确保无速度信息

        elif mode == "Turn2Straight":
            # # 先停止转弯
            # self.clear_driver(times / 2, steps, control_driver)
            # self.radius = 0
            # self.omega = 0
            # # 然后开始直行
            # self.start_driver("Straight", times / 2, steps, value, control_driver)

            # 另外一种方案，直接同时递减omega递增speed
            omega_step = self.omega / steps
            speed_step = value / steps
            for i in range(steps):
                self.omega -= omega_step
                self.speed += speed_step
                self.set_driver(control_driver, times / steps)
            pass
            self.omega = 0
            self.radius = 0

        elif mode == "Turn2Turn":
            # 先递减原来的，再递增新的

            # self.clear_driver(times/8, steps, control_driver)
            # # self.omega = 0
            # self.design_radius()
            # self.start_driver("Turn", times*7/8, steps, value, control_driver)

            # 直接变成新的转弯
            self.design_radius()
            self.omega = value
            self.set_driver(control_driver, 0.1)
            pass
        return

    # 前进后退
    def action_forward_back(self, control_driver):
        # 转弯变直行
        speed_default = 0.2
        time_change_default = 0.1
        step_numbers_default = 50000
        if self.omega != 0:
            if self.expect_x < 0:
                print("Action mode: Turn2Backward\n")
                self.interpolation("Turn2Straight", time_change_default, step_numbers_default, -speed_default, control_driver)
            else:
                print("Action mode: Turn2Forward\n")
                self.interpolation("Turn2Straight", time_change_default, step_numbers_default, speed_default, control_driver)
        # 后退
        elif self.expect_x < 0:
            if self.speed == 0:
                print("Action mode: Start Backward\n")
                self.start_driver("Straight", time_change_default, step_numbers_default, -speed_default, control_driver)
            elif self.speed > 0:
                print("Action mode: Forward2Backward\n")
                self.interpolation("Straight", time_change_default, step_numbers_default, -speed_default, control_driver)
            elif self.speed < 0:
                print("Action mode: Moving Backward\n")
        # 直行
        elif self.expect_x > 0:
            if self.speed == 0:
                print("Action mode: Start Forward\n")
                self.start_driver("Straight", time_change_default, step_numbers_default, speed_default, control_driver)
            elif self.speed < 0:
                print("Action mode: Backward2Forward\n")
                self.interpolation("Straight", time_change_default, step_numbers_default, speed_default, control_driver)
            elif self.speed > 0:
                print("Action mode: Moving Forward\n")
        pass

    # 旋转
    def action_rotation(self, control_driver):
        pass

    # 前进转弯
    def action_forward_and_turning(self, control_driver):
        # 直行状态转转弯
        omega_default = 0.2
        time_change_default = 0.1
        step_numbers_default = 2500
        if self.speed != 0:
            self.design_radius()
            if self.expect_theta < 0:
                print("Action mode: Straight2Turn_R\n")
                self.interpolation("Straight2Turn", time_change_default, step_numbers_default, -omega_default, control_driver)
            else:
                print("Action mode: Straight2Turn_L\n")
                self.interpolation("Straight2Turn", time_change_default, step_numbers_default, omega_default, control_driver)
        # 起步转弯
        elif self.omega == 0:
            self.design_radius()
            if self.expect_theta < 0:
                print("Action mode: Start Turning_R\n")
                self.start_driver("Turn", time_change_default, step_numbers_default, -omega_default, control_driver)
            else:
                print("Action mode: Start Turning_L\n")
                self.start_driver("Turn", time_change_default, step_numbers_default, omega_default, control_driver)
        # 转弯转转弯
        else:
            abs_theta = abs(self.expect_theta)
            if abs_theta <= 55:
                temp_radius = self.robot_r * 2 - ((abs_theta - 15) / (55 - 15) * self.robot_r)
            elif 85 < abs_theta <= 90:
                temp_radius = self.robot_r / 2
            # 保持原状态的转弯
            if temp_radius == self.radius and self.omega * self.expect_theta > 0:
                if self.expect_theta < 0:
                    print("Action mode: Turning_R\n")
                else:
                    print("Action mode: Turning_L\n")

            else:
                if self.expect_theta < 0:
                    print("Action mode: Turn2Turn_R, radius:", temp_radius, "->", self.radius)
                    self.interpolation("Turn2Turn", time_change_default, step_numbers_default, -omega_default, control_driver)
                else:
                    print("Action mode: Turn2Turn_L, radius:", temp_radius, "->", self.radius)
                    self.interpolation("Turn2Turn", time_change_default, step_numbers_default, omega_default, control_driver)
        print("omega:", self.omega)
        pass


if __name__ == '__main__':
    pc = PositionControl2()
