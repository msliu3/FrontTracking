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

import os
import sys

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

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
        port_name = detect_serials(description="ttyACM1")  # Arduino Mega 2560 ttyACM0
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

        self.pwd = os.path.abspath(os.path.abspath(__file__))
        self.father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
        self.textnum = 0
        self.lock_value = 0

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

    # def brake_control(self, pull_down=False, side='B', distance=0):
    #     # Command format: BR500 - Brake Right for 500
    #     #                 RB500 - Release Both for 500
    #     command = str()
    #     if pull_down:
    #         if not self.is_locked:
    #             command = 'B' + side + str(distance)
    #             # self.read_softskin_data()
    #             self.is_locked = True
    #     else:
    #         if self.is_locked:
    #             command = 'R' + side + str(distance)
    #             # self.read_softskin_data()
    #             self.is_locked = False
    #         pass
    #     self.serial.write(bytes(command, encoding='utf-8'))
    #     self.serial.flushOutput()

    def brake_control(self, up_down=False,  distance=0, side='B'):
        # Command format: BR500 - Brake Right for 500
        #                 RB500 - Release Both for 500
        command = str()
        if up_down:
            if self.lock_value + distance <= 550:
                command = 'B' + side + str(distance)
                # self.read_softskin_data()
                self.lock_value += distance
            else:
                print("lock out of range")
        else:
            if self.lock_value - distance >= 0:
                command = 'R' + side + str(distance)
                self.lock_value -= distance
            else:
                print("lock out of range")
            # self.read_softskin_data()
        print(self.lock_value, '\t', command)
        self.serial.write(bytes(command, encoding='utf-8'))
        print("finish")
        self.serial.flushOutput()


    def detect_accident(self, cd, using=True):
        """using 代表用户正在使用"""
        """压力刹车的值"""
        flag_pre_brake = False
        """运行之前应运行baselinebuild"""
        add = np.ones(len(self.base_data)).reshape(1, -1)
        front_part_list = [0, 0, 0, 1, 1, 1, 1,
                           1, 1, 1, 0, 0, 0]
        front_part_list = np.array(front_part_list)
        """计算变化速率用"""
        slope_matrix = np.zeros((1, len(self.base_data)))  # 储存原始数据
        slope_alarm = np.zeros((3, len(self.base_data)))  # 储存斜率
        add_slope = np.ones((1, 3))  # 计算用
        slope_number = 5  # 原始数据储存的组数
        for i in range(slope_number):  # 初始化储存原始数据的矩阵
            self.serial.flushInput()
            self.read_softskin_data(0)
            while len(self.raw_data) != len(self.base_data):
                print("incorrect raw_data")
                self.read_softskin_data(0)
            slope_matrix = np.insert(slope_matrix, 0, self.raw_data, 0)
        slope_matrix = np.delete(slope_matrix, -1, axis=0)

        data_path = self.father_path + os.path.sep + "resource" + os.path.sep + "testS" + str(self.textnum) + ".txt"

        with open(data_path, 'w') as file:
            while using:
                self.serial.flushInput()
                self.read_softskin_data(0)
                if len(self.raw_data) == len(self.base_data):
                    temp_data = np.array(self.raw_data) - np.array(self.base_data)
                    file.write(str(temp_data.tolist())+'\n')
                    """检测有没有前趴"""
                    temp_data[temp_data < 30] = 0
                    temp_data[temp_data >= 30] = 1
                    temp_sum = add.dot(temp_data)
                    front_sum = front_part_list.dot(temp_data)
                    if temp_sum > 4 and front_sum > 0:
                        cd.omega = 0
                        cd.speed = 0
                        cd.radius = 0
                        print("Several sensors pressed:\t", "All:", temp_sum, "\tFront:", front_sum)
                        self.brake_control(True, 550 - self.lock_value)
                        break
                    #  检测变化率
                    temp_data = np.array(self.raw_data) - np.array(self.base_data)
                    slope_matrix = np.insert(slope_matrix, 0, self.raw_data, 0)
                    slope_matrix = np.delete(slope_matrix, -1, axis=0)
                    # print(slope_matrix)
                    slope_every_sensor = slope_matrix[0, :] - slope_matrix[-1, :]
                    # print(slope_every_sensor)
                    slope_threshold = 45
                    slope_every_sensor[slope_every_sensor < slope_threshold] = 0
                    slope_every_sensor[slope_every_sensor >= slope_threshold] = 1
                    # print(slope_every_sensor)
                    slope_alarm[0:-1, :] = slope_alarm[1:slope_alarm.shape[0], :]
                    slope_alarm[-1, :] = slope_every_sensor
                    # print(slope_alarm)
                    slope_every_sensor = add_slope.dot(slope_alarm)
                    # print(slope_every_sensor)
                    # print(type(slope_every_sensor))
                    # print(temp_data.max())
                    if slope_every_sensor.max() >= slope_alarm.shape[0]:
                        pos = np.unravel_index(np.argmax(slope_every_sensor), slope_every_sensor.shape)
                        print("Sudden change in sensor:", pos)
                        cd.omega = 0
                        cd.speed = 0
                        cd.radius = 0
                        self.brake_control(True, 550 - self.lock_value)
                        break
                    """检测最大压力"""
                    max_pressure = temp_data.max()
                    threshold_pre = 120
                    threshold_fin = 180
                    if not flag_pre_brake:
                        if max_pressure < threshold_pre:
                            pass
                        elif max_pressure < threshold_fin:
                            print("yushache")
                            self.brake_control(True, 150)
                            flag_pre_brake = True
                            time.sleep(1)
                        else:
                            print("Max_pressure:", max_pressure)
                            self.brake_control(True, 550)
                            break
                    else:
                        if max_pressure < threshold_pre:
                            print("cancel yushache")
                            self.brake_control(False, 150)
                            flag_pre_brake = False
                            time.sleep(1)
                        elif max_pressure < threshold_fin:
                            pass
                        else:
                            print("Max_pressure:", max_pressure)
                            self.brake_control(True, 400)
                            break

            file.close()


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
                    self.brake_control(True, 550 - self.lock_value)
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
                    self.brake_control(True, 550 - self.lock_value)
                    self.is_locked = True
                    self.read_softskin_data(0)
                    break

    def unlock(self):
        lock_a = 0
        lock_b = 0
        lock_c = 0
        add = np.ones(len(self.base_data)).reshape(1, -1)

        while True:
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
                self.brake_control(False, self.lock_value)
                time.sleep(2)
                break

    def adjust_direction(self, cd, event_ssl):
        """using 代表用户不在使用"""
        """运行之前应运行baselinebuild"""
        add = np.ones(len(self.base_data)).reshape(1, -1)
        radius_list = [-90, -110, -120, -135, -155, -165, 180,
                       165, 155, 135, 120, 110, 90]
        radius_error = [-20, -15, -15, -10, -10, 0, 14, 0, 10, 10, 15, 15, 20]
        radius_list = np.array(radius_list)
        radius_error = np.array(radius_error)
        radius_list = radius_list + radius_error
        omega_default = 0.35
        while True:
            self.serial.flushInput()
            self.read_softskin_data(0)
            if len(self.base_data) == len(self.raw_data):
                temp_data = np.array(self.raw_data) - np.array(self.base_data)
                max_pressure = temp_data.max()
                if max_pressure > 200:
                    cd.speed = 0
                    cd.omega = 0
                    cd.radius = 0
                    event_ssl.clear()
                    self.locking = True
                    break
                else:
                    temp_data[temp_data < 40] = 0
                    temp_data[temp_data >= 40] = 1
                    temp_sum = add.dot(temp_data)
                    # print(temp_sum)
                    if temp_sum == 1:
                        radius_temp = temp_data.dot(radius_list)
                        print(radius_temp)
                        cd.speed = 0
                        cd.omega = 0
                        cd.radius = 0
                        event_ssl.clear()
                        time.sleep(1)
                        cd.speed = -0.1
                        time.sleep(2)
                        cd.speed = 0
                        cd.radius = 0
                        expectedTHETA = cd.position[2] + math.radians(radius_temp)
                        if expectedTHETA > math.pi:
                            expectedTHETA -= 2 * math.pi
                        elif expectedTHETA <= -math.pi:
                            expectedTHETA += 2 * math.pi
                        cd.omega = radius_temp / (abs(radius_temp)) * omega_default
                        # time.sleep(2)
                        while True:
                            time.sleep(0.1)
                            print("time")
                            if abs(cd.position[2] - expectedTHETA) <= 0.2:
                                print("arrived!")
                                break
                        cd.omega = 0
                        cd.speed = -0.2
                        if abs(radius_temp) < 100:
                            time.sleep(2)
                        elif abs(radius_temp) < 130:
                            time.sleep(3)
                        elif abs(radius_temp) < 160:
                            time.sleep(3.5)
                        else:
                            time.sleep(4)
                        cd.speed = 0

                        # time.sleep()
                        # self.serial.flushOutput()
                        # print("test",self.lock_value)
                        # self.brake_control(True, 550 - self.lock_value)

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




