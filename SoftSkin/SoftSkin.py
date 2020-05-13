#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   SoftSkin.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/3/4 23:50   msliu      1.0

@Description
------------
1. 通过串口，读取Arduino板的数据
2. 对数据进行处理和抽象
    抽象成基本的机器人行为：
        前进，后退，左转，右转
3.结合ControlDriver去控制walker实际运行
"""

import serial
import serial.tools.list_ports
import numpy as np
import DigitalDriver.ControlDriver as CD
import time
import math
import threading


def print_serial(port):
    print("---------------[ %s ]---------------" % port.name)
    print("Path: %s" % port.device)
    print("Descript: %s" % port.description)
    print("HWID: %s" % port.hwid)
    if not None == port.manufacturer:
        print("Manufacture: %s" % port.manufacturer)
    if not None == port.product:
        print("Product: %s" % port.product)
    if not None == port.interface:
        print("Interface: %s" % port.interface)
    print()


def detect_serials(description="target device", vid=0x10c4, pid=0xea60):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print_serial(port)

        if port.description.__contains__(description):
            port_path = port.device
            return port_path
        else:
            print("Cannot find the target device: %s" % description)
    return None


class SoftSkin(object):
    def __init__(self):
        port_name = detect_serials(description="ttyACM0")  # Arduino Mega 2560 ttyACM0
        baud_rate = 115200
        print(port_name, baud_rate)
        self.serial = serial.Serial(port_name, baud_rate, timeout=None)
        # self.serial = serial.Serial(port_name, baud_rate, timeout=0.3)
        self.raw_data = []  # 保存一帧数据
        self.base_data = []  # 建立一组基准值用于初始化

        self.left_skin = False
        self.front_skin = False
        self.right_skin = False

        self.label_dict = {0: [0, "left_right"],
                           1: [1, "left_forward"],
                           2: [2, "left_left"],
                           10: [10, "right_left"],
                           11: [11, "right_forward"],
                           12: [12, "right_right"],
                           100: "No_data"}
        self.label = ""

        self.is_locked = False
        self.locking = False
        pass

    def read_softskin_data(self, flag_show=1):
        """
        一组数据包含13个数据，以","分隔。
        物理意义不明确

        数据不一定稳定，可能出现空值
        """
        try:
            one_line_data = self.serial.readline().decode("utf-8")
            one_line_data = one_line_data.split('\r')[0]
            one_line_data = one_line_data.split(',')
            one_line_data = list(map(int, one_line_data))
            if flag_show:
                print(one_line_data)
            self.raw_data = one_line_data
            return one_line_data
        except BaseException as be:
            print("Data error", be)
            self.raw_data = []


    def build_base_line_data(self, initial_size=15):
        """
        1.建立一组基准数值
            检测异常值
            取平均值
        :return:
        """
        base_list = []
        for i in range(initial_size):
            self.read_softskin_data(0)
            if len(self.raw_data) == 13:
                temp_raw_data = self.raw_data
                # temp_max = max(temp_raw_data)
                # temp_min = min(temp_raw_data)
                # if temp_min * 2 < temp_max:
                #     continue
                # else:
                #     base_list += temp_raw_data
                base_list += temp_raw_data
        mean_base_list = np.array(base_list).reshape([-1, 13])
        add_col = np.ones(mean_base_list.shape[0]).reshape([1, -1])
        mean_base_list = add_col.dot(mean_base_list) / mean_base_list.shape[0]
        self.base_data = mean_base_list.tolist()[0]
        self.base_data = list(map(lambda x: int(x) - 1, self.base_data))
        print("base line data: ", self.base_data)
        pass

    def detect_three_point(self):
        """
        检测三个方向的触发
        :return:
        """
        self.read_softskin_data(0)
        normalize = np.array(self.raw_data) - np.array(self.base_data)
        # print(normalize)
        if np.max(normalize[0:3]) > 20:
            self.left_skin = True
        if np.max(normalize[3:-4]) > 20:
            self.front_skin = True
        if np.max(normalize[-3:]) > 20:
            self.right_skin = True
        print(self.left_skin, self.front_skin, self.right_skin)
        pass

    def clear_skin_state(self):
        self.left_skin = False
        self.front_skin = False
        self.right_skin = False
        pass

    def basical_control(self, control_driver, setup=(1, 0.4, 56 / 2), sit_mode=False):
        """
        根据soft skin被按下的状态，将控制分解为，前行，后退，左转，右转
        前行：
            左右扶手被同时按下
        后退：
            前扶手被按下
        左转：
            左扶手被按下
        右转
            右扶手被按下
        :param control_driver:
        :return:
        """
        if self.left_skin and self.right_skin:
            print("forward")
            control_driver.speed = setup[0]
            control_driver.omega = 0
            control_driver.radius = 0
        elif self.front_skin:
            print("back")
            control_driver.speed = -setup[0]
            control_driver.omega = 0
            control_driver.radius = 0
        elif self.left_skin:
            print("turning left")
            if not sit_mode:
                control_driver.speed = 0
                control_driver.omega = setup[1]
                control_driver.radius = setup[2]
            else:
                control_driver.speed = 0
                control_driver.omega = -setup[1]
                control_driver.radius = setup[2]
        elif self.right_skin:
            print("turning right")
            if not sit_mode:
                control_driver.speed = 0
                control_driver.omega = -setup[1]
                control_driver.radius = setup[2]
            else:
                control_driver.speed = 0
                control_driver.omega = setup[1]
                control_driver.radius = setup[2]
        elif not (self.left_skin and self.front_skin and self.right_skin):
            print("stop")
            control_driver.speed = 0
            control_driver.omega = 0
            control_driver.radius = 0

    def detect_all_point(self, power=30):
        """
        检测全方向的触发
        返回列表，值为0,1
        :return:
        """
        self.read_softskin_data()
        normalize = np.array(self.raw_data) - np.array(self.base_data)
        normalize[normalize < power] = 0
        normalize[normalize >= power] = 1
        return normalize

    def label_front_follow(self):
        self.build_base_line_data()
        while True:
            np_data = self.detect_all_point()
            b = np.arange(len(np_data))
            # print(b[np_data == 1])
            if len(b[np_data == 1]) == 1:
                self.label = self.label_dict[b[np_data == 1][0]][1]
                # print(self.label)
            else:
                self.label = self.label_dict[100]
                # print(self.label)

    def brake_control(self, command=False):
        if command:
            if not self.is_locked:
                self.serial.write(bytes('B', encoding='utf-8'))
                # self.read_softskin_data()
                self.is_locked = True
        else:
            if self.is_locked:
                self.serial.write(bytes('R', encoding='utf-8'))
                # self.read_softskin_data()
                self.is_locked = False
            pass
        self.serial.flushOutput()

    def detect_accident(self, cd, using = True):
        """using 代表用户正在使用"""
        """运行之前应运行baselinebuild"""
        add = np.ones(len(self.base_data)).reshape(1, -1)
        front_part_list = [0, 0, 0, 1, 1, 1, 1,
                           1, 1, 1, 0, 0, 0]
        front_part_list = np.array(front_part_list)
        self.serial.flushInput()
        while using:
            self.read_softskin_data(0)
            if len(self.raw_data) == len(self.base_data):
                temp_data = np.array(self.raw_data) - np.array(self.base_data)
                max_pressure = temp_data.max()
                if max_pressure > 165:
                    self.locking = True
                    print(max_pressure)
                    break
                else:
                    temp_data[temp_data < 15] = 0
                    temp_data[temp_data >= 15] = 1
                    temp_sum = add.dot(temp_data)
                    front_sum = front_part_list.dot(temp_data)
                    if temp_sum > 4 and front_sum > 0:
                        cd.omega = 0
                        cd.speed = 0
                        cd.radius = 0
                        print(temp_sum, front_sum)
                        self.locking = True
                        self.brake_control(self.locking)
                        self.is_locked = True
                        break

    def stop_ssl(self, SSLrunning, cd, event_ssl):
        add = np.ones(len(self.base_data)).reshape(1, -1)

        while SSLrunning:
            self.serial.flushInput()  # 清空上一次的Softskin的输入，否则会被继续当成输入
            self.read_softskin_data(0)
            if len(self.raw_data) == len(self.base_data):
                temp_data = np.array(self.raw_data) - np.array(self.base_data)
                temp_data[temp_data < 15] = 0
                temp_data[temp_data >= 15] = 1
                temp_sum = add.dot(temp_data)
                if temp_sum > 0:
                    self.locking = True
                    print("SSL stop! Walker ready!")
                    cd.omega = 0
                    cd.speed = 0
                    cd.radius = 0
                    # self.brake_control(self.locking)
                    event_ssl.clear()
                    break

    def lock(self, cd, SSLrunning = True):
        add = np.ones(len(self.base_data)).reshape(1, -1)
        self.serial.flushInput()
        while SSLrunning:
            self.read_softskin_data(0)
            if len(self.raw_data) == len(self.base_data):
                temp_data = np.array(self.raw_data) - np.array(self.base_data)
                temp_data[temp_data < 15] = 0
                temp_data[temp_data >= 15] = 1
                temp_sum = add.dot(temp_data)
                if temp_sum > 0:
                    cd.omega = 0
                    cd.speed = 0
                    cd.radius = 0
                    self.locking = True
                    print("SSL stop! Walker ready!")
                    self.brake_control(self.locking)
                    self.is_locked = True
                    self.read_softskin_data(0)
                    break

    def unlock(self):
        lock_a = 0
        lock_b = 0
        lock_c = 0
        add = np.ones(len(self.base_data)).reshape(1, -1)

        while self.is_locked:
            self.serial.flushInput()
            # print("processing")
            self.read_softskin_data(0)
            if len(self.raw_data) != len(self.base_data):
                continue
            temp_data = np.array(self.raw_data) - np.array(self.base_data)
            temp_data[temp_data < 15] = 0
            temp_data[temp_data >= 15] = 1
            temp_sum = add.dot(temp_data)
            # position = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
            # position = np.array(position)
            # position = position.dot(temp_data)
            # print(temp_sum, position)
            if temp_data[0] == 1 and temp_sum == 1 and not lock_a:
                lock_a = 1
                print("LOCK A unlocked")
            if temp_data[-1] == 1 and temp_sum == 1 and not lock_b:
                lock_b = 1
                print("LOCK B unlocked")
            if temp_data[6] == 1 and temp_sum == 1 and not lock_c:
                lock_c = 1
                print("LOCK C unlocked")
            if lock_a * lock_b * lock_c == 1:
                self.locking = False
                print("All unlocked!")
                self.brake_control(self.locking)
                break

    def adjust_direction(self, control_driver, using=False):
        """using 代表用户不在使用"""
        """运行之前应运行baselinebuild"""
        add = np.ones(len(self.base_data)).reshape(1, -1)
        radius_list = [-90, -110, -120, -135, -155, -165, 180,
                       165, 155, 135, 120, 110, 90]
        radius_list = np.array(radius_list)
        omega_default = 0.3
        control_driver.speed = 0
        control_driver.radius = 0
        while not using:
            if len(self.raw_data) != len(self.base_data):
                continue
            temp_data = np.array(self.raw_data) - np.array(self.base_data)
            max_pressure = temp_data.max()
            if max_pressure > 100:
                self.locking = True
                continue
            else:
                temp_data[temp_data < 20] = 0
                temp_data[temp_data >= 20] = 1
                temp_sum = add.dot(temp_data)
                # print(temp_sum)
                if temp_sum == 1:
                    radius_temp = temp_data.dot(radius_list)
                    control_driver.omega = radius_temp / (abs(radius_temp)) * omega_default
                    time_for_adjust = math.radians(abs(radius_temp)) / omega_default
                    print(radius_temp, time_for_adjust)
                    time.sleep(time_for_adjust)
                    control_driver.omega = 0
                    self.locking = 1
                    break


if __name__ == '__main__':
    softskin = SoftSkin()

    # 如果需要使用sit模式，先将轮机翻转，然后将sit_mode=True
    control_driver = CD.ControlDriver(left_right=1)
    control_driver.start()
    # softskin.build_base_line_data()
    # while True:
    #     # 移动控制
    #     # softskin.detect_three_point()
    #     # softskin.basical_control(control_driver, sit_mode=True)
    #     # softskin.clear_skin_state()
    #
    #     # 数据采集
    #     data = softskin.detect_all_point()
    #     print(data)
    softskin.label_front_follow()




