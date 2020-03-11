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
        baud_rate = 9600
        print(port_name, baud_rate)
        self.serial = serial.Serial(port_name, baud_rate, timeout=None)

        self.raw_data = []  # 保存一帧数据
        self.base_data = []  # 建立一组基准值用于初始化

        self.left_skin = False
        self.front_skin = False
        self.right_skin = False
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
        except BaseException as be:
            print("Data error", be)
            self.raw_data = []
        pass

    def build_base_line_data(self, initial_size=15):
        """
        1.建立一组基准数值
            检测异常值
            取平均值
        :return:
        """
        base_list = []
        for i in range(initial_size):
            self.read_softskin_data()
            if len(self.raw_data) == 13:
                temp_raw_data = self.raw_data
                temp_max = max(temp_raw_data)
                temp_min = min(temp_raw_data)
                if temp_min * 2 < temp_max:
                    continue
                else:
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

    def detect_all_point(self):
        """
        检测全方向的触发
        返回列表，值为0,1
        :return:
        """
        self.read_softskin_data(0)
        normalize = np.array(self.raw_data) - np.array(self.base_data)
        normalize[normalize < 20] = 0
        normalize[normalize > 20] = 1
        return normalize


if __name__ == '__main__':
    softskin = SoftSkin()
    # 如果需要使用sit模式，先将轮机翻转，然后将sit_mode=True
    control_driver = CD.ControlDriver(left_right=1)
    control_driver.start()
    softskin.build_base_line_data()
    while True:
        # 移动控制
        # softskin.detect_three_point()
        # softskin.basical_control(control_driver, sit_mode=True)
        # softskin.clear_skin_state()

        # 数据采集
        data = softskin.detect_all_point()
        print(data)
