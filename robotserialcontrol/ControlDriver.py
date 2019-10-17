#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   ControlDriver.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/16 21:46   msliu      1.0      

@Description
------------
接受一个线速度v和角速度w
根据线速度和角速度，结算实际转速(RPM-Revolutions Per Minute)


怎么隐隐约约觉得这好像是一个互斥问题啊

"""
import time

import robotserialcontrol.DigitalServoDriver as DsD
import serial




class ControlDriver(object):

    def __init__(self, radius_wheel=52.55):
        driver = DsD.DigitalServoDriver()
        baud_rate = driver.baud_rate
        self.ser_l = serial.Serial(driver.left, baud_rate, timeout=None)
        self.ser_r = serial.Serial(driver.right, baud_rate, timeout=None)

        return


    def get_rpm(self, speed):
        rpm_hex = int(speed / 6000 * 16384)
        # print(rpm_hex)
        rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        return rpm

    def control_part(self):
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xc4, 0xc6]
        rpm = [0x06, 0x00, 0x88, 0x8e]
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(start))
        self.ser_r.write(bytes(start))
        self.ser_l.write(bytes(pc_mode))
        self.ser_r.write(bytes(pc_mode))

        while True:
            temp = self.get_rpm(200)
            rpm[1] = temp[0]
            rpm[2] = temp[1]
            rpm.pop(3)
            sum = 0
            for item in rpm:
                sum = sum+item
            rpm.append(sum)
            print(rpm)
            self.ser_l.write(bytes(rpm))
            self.ser_r.write(bytes(rpm))
            time.sleep(0.5)
            watch = [0x80, 0x00, 0x80]
            self.ser_l.write(bytes(watch))
            self.ser_r.write(bytes(watch))
    pass


if __name__ == '__main__':
    cd = ControlDriver()
    # a, b = cd.get_rpm(200)
    # print(a, bytes(b))
    cd.control_part()