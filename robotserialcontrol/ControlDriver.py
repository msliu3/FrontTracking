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


def singleton(cls, *args, **kw):
    instances = {}

    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]

    return _singleton


# @singleton
class ControlDriver(object):

    def __init__(self, radius_wheel=52.55, flag_end=0):
        driver = DsD.DigitalServoDriver()
        baud_rate = driver.baud_rate
        self.ser_l = serial.Serial(driver.left, baud_rate, timeout=None)
        self.ser_r = serial.Serial(driver.right, baud_rate, timeout=None)
        self.flag_end = flag_end
        self.speed = 50
        return

    def get_rpm(self):
        rpm_hex = int(self.speed / 6000 * 16384)
        # print(rpm_hex)
        rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        return rpm

    @staticmethod
    def control_part(self):
        print("start control part")
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xc4, 0xc6]
        rpm = [0x06, 0x00, 0x88, 0x8e]
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(start))
        self.ser_r.write(bytes(start))
        self.ser_l.write(bytes(pc_mode))
        self.ser_r.write(bytes(pc_mode))

        while True:
            temp = self.get_rpm()
            rpm[1] = temp[0]
            rpm[2] = temp[1]
            rpm.pop(3)
            sum = 0
            for item in rpm:
                sum = sum + item
            rpm.append(sum)
            print(rpm)
            self.ser_l.write(bytes(rpm))
            self.ser_r.write(bytes([0x06, 0x00, 0x88, 0x8e]))
            time.sleep(0.5)
            watch = [0x80, 0x00, 0x80]
            self.ser_l.write(bytes(watch))
            self.ser_r.write(bytes(watch))
            if self.flag_end != 0:
                break

        self.ser_l.write(bytes(end))
        self.ser_r.write(bytes(end))

    pass


# cd = ControlDriver()

if __name__ == '__main__':
    from multiprocessing import Process
    import multiprocessing


    # pool = multiprocessing.Pool(processes=1)
    def loop(control):
        # while True:
        print("start loop")
        time.sleep(10)
        print("end")
        # control.flag_end = 1
        # temp = input("num")
        # if temp == -1:
        #     control.flag_end = 1
        #     break
        # control.speed = temp


    # pool.apply(func=loop(cd))
    # pool.apply(func=cd.control_part())
    # pool.close()
    # pool.join()
    #
    # p = Process(target=loop(cd))
    cd = ControlDriver()
    p2 = Process(target=ControlDriver.control_part, args=())
    # p.start()
    p2.start()
    # cd.control_part()
