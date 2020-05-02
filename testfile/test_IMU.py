#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   test_IMU.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/1/15 6:20   msliu      1.0      

@Description
------------
None
"""
import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

import time
from threading import Thread
from DigitalDriver import DigitalServoDriver_linux as DsD
from DigitalDriver import DriverMonitor_zhuzhi as DM
from DigitalDriver import WheelEncoderOdometry as odo
import serial
import math


class ControlDriver(Thread):

    def __init__(self, V=0.0, OMEGA=0.0, record_mode=False, left_right=1):
        """
        :param radius_wheel:
        :param record_mode:
        :param radius:
        :param left_right:
            如果发现 左右轮数据反了
            将 0 改为 1 或 1 改为 0
        """
        Thread.__init__(self)
        self.radius_wheel = 85.00   #车轮半径/mm
        self.wheel_base = 540.00    #轮距/mm
        self.record_mode = record_mode
        self.speed = V  #线速度
        self.omega = OMEGA  #角速度
        self.position = [0.0, 0.0, 0.0]
        self.count = 0
        self.is_stopped = record_mode
        driver = DsD.DigitalServoDriver()
        self.left_right = left_right
        baud_rate = driver.baud_rate
        if left_right == 1:
            self.ser_l = serial.Serial(driver.left, baud_rate, timeout=0.05)    #左轮串口
            self.ser_r = serial.Serial(driver.right, baud_rate, timeout=0.05)   #右轮串口
        else:
            self.ser_l = serial.Serial(driver.right, baud_rate, timeout=0.05)  # 左轮串口
            self.ser_r = serial.Serial(driver.left, baud_rate, timeout=0.05)  # 右轮串口
        # self.MCU = serial.Serial("/dev/ttyACM0", baud_rate=9600, timeout=1) #Arduino
        self.monitor_l = DM.DriverMonitor()
        self.monitor_r = DM.DriverMonitor()
        self.plot_x = [0.0]
        self.plot_y = [0.0]

        # 初始化时读取一次驱动器监控信息，记录初始时encoder位置
        read_byte_l = self.read_monitor(self.ser_l)
        read_byte_r = self.read_monitor(self.ser_r)

        # 初始化Odometry
        self.motorStatus_l = self.monitor_l.processData(read_byte_l)
        self.motorStatus_r = self.monitor_r.processData(read_byte_r)
        print('-------------------------------------------------------------------------------------------------------')
        print('Initial LEFT monitor: ', self.motorStatus_l)
        print('Initial RIGHT monitor:', self.motorStatus_r)
        Odo_l_init = self.motorStatus_l['FeedbackPosition']
        Odo_r_init = self.motorStatus_r['FeedbackPosition']
        print('init: ', Odo_l_init, Odo_r_init)
        print('-------------------------------------------------------------------------------------------------------')
        self.odo = odo.Odometry(X=0.0, Y=0.0, THETA=0.0, Odo_l=Odo_l_init, Odo_r=Odo_r_init, plot=False)
        # time.sleep(2)

    def read_monitor(self, ser):
        ser.write(bytes([0x80, 0x00, 0x80]))
        read_byte = ser.read(5)
        if read_byte[4] == 0x80:
            read_byte += ser.read(31)
        else:
            read_byte += ser.read(27)
        return read_byte

    def change_speed(self, v, omega):
        if(self.speed + self.omega != 0) and (v + omega == 0):
            self.stop_motor()
        elif( (v + omega) != 0 and (self.speed + self.omega == 0)):
            self.start_motor()
        self.speed = v
        self.omega = omega
        time.sleep(0.05)

    def get_wheel_speed(self):
        # 计算两轮线速度
        # v_r = (2V + omega * wheelbase) / 2
        # v_l = (2V - omega * wheelbase) / 2
        vl = (2*self.speed - self.omega * self.wheel_base / 1000) / 2
        vr = (2*self.speed + self.omega * self.wheel_base / 1000) / 2
        return vl, vr

    def speed2rpm(self, speed):
        # 线速度 --> 角速度
        rpm = speed / (2 * math.pi * self.radius_wheel / 1000) * 60
        return int(rpm)

    def rpm2byte(self, rpm):
        rpm_byte = [0x06, 0x00, 0x88, 0x8e]
        rpm_hex = int(rpm / 6000 * 16384)
        if rpm_hex >= 0:
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        else:
            temp = 0xFFFF
            rpm_hex = temp + rpm_hex
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        rpm_byte[1] = rpm[0]
        rpm_byte[2] = rpm[1]
        rpm_byte.pop(3)
        last = 0
        for item in rpm_byte:
            last = last + item
        if last > 256:
            last = last & 0xFF
        rpm_byte.append(last)
        return rpm_byte

    def control_part_speedmode(self):
        print("\n===================================== Start speed control ! =====================================")
        self.start_motor()
        # 如果 record_mode 是 True，则停掉电机，只记录数据
        if self.record_mode:
            self.stop_motor()

        while True:
            vl, vr = self.get_wheel_speed()
            # print("left: ", vl, "; right: ", vr)
            vl = self.speed2rpm(vl)
            vr = self.speed2rpm(vr)
            left = self.rpm2byte(-vl)
            right = self.rpm2byte(vr)
            # print("byte_left: ", left, "byte_right: ", right)
            self.ser_l.write(bytes(left))
            self.ser_l.flush()
            self.ser_l.read(2)
            self.ser_r.write(bytes(right))
            self.ser_r.flush()
            self.ser_r.read(2)
            time.sleep(0.05)
            try:
                read_byte_l = self.read_monitor(self.ser_l)
                read_byte_r = self.read_monitor(self.ser_r)

                Odo_l = self.motorStatus_l['FeedbackPosition']
                Odo_r = self.motorStatus_r['FeedbackPosition']

                # 更新位置
                self.position = self.odo.updatePose(-Odo_l, Odo_r)
                # print('Position:  X=', self.position[0], 'm;  Y=', self.position[1], 'm; THETA=', self.position[2] / math.pi * 180, '°;')

                if math.sqrt((self.position[0] - self.plot_x[-1]) ** 2 + (self.position[1] - self.plot_y[-1]) ** 2) > 0.1:
                    self.plot_x.append(self.position[0])
                    self.plot_y.append(self.position[1])

                # 若有故障
                if self.motorStatus_l["Malfunction"] or self.motorStatus_r["Malfunction"]:
                    # print('Left motor malfunction:  ' + self.motorStatus_l["Malfunction"])
                    # print('Right motor malfunction: ' + self.motorStatus_r["Malfunction"])
                    self.flag_end = 1

                # print("%f\t%f\t%f\t%f" % (time.time(), math.degrees(self.odo.THETA),self.position[0],self.position[1]))
                # print(
                #     "%f\t%f\t%f\t%f\t%f" % (
                #     time.time(), self.odo.get_dxdydtheta()[0], self.odo.get_dxdydtheta()[1], self.odo.getROS_XYTHETA()[0],
                #     self.odo.getROS_XYTHETA()[1]))
            except IndexError as i:
                print(i,"except")

            self.ser_l.reset_input_buffer()
            self.ser_r.reset_input_buffer()
        pass

    def start_motor(self):
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xc4, 0xc6]
        self.ser_l.write(bytes(start))
        self.ser_l.read(2)
        self.ser_r.write(bytes(start))
        self.ser_r.read(2)
        self.ser_l.write(bytes(pc_mode))
        self.ser_l.read(2)
        self.ser_r.write(bytes(pc_mode))
        self.ser_r.read(2)
        self.is_stopped = False

    def stop_motor(self):    #关闭电机，同时关闭刹车
        self.speed = 0
        self.omega = 0
        time.sleep(0.05)
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(end))
        self.ser_l.read(2)
        self.ser_r.write(bytes(end))
        self.ser_r.read(2)
        # 读取一帧驱动器监控信息
        read_byte_l = self.read_monitor(self.ser_l)
        read_byte_r = self.read_monitor(self.ser_r)

        self.is_stopped = True

    def ext_brake(self):
        self.speed = 0
        self.omega = 0
        pass

    def run(self):
        self.control_part_speedmode()

    pass

if __name__ == "__main__":
    cd = ControlDriver(left_right=1, V = 0.0, OMEGA=-0.1, record_mode=False)
    cd.start()

