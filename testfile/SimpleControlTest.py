#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   SimpleControlTest.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Description
------------      -------    --------    -----------
2019/10/15 22:06   msliu      1.0

1.打开串口
2.启动伺服器
3.调整至速度模式选择——PC数字输入
4.输入速度
5.电机停止

"""

import time

import serial

import robotserialcontrol.DigitalServoDriver as dsd

DigitalDriver = dsd.DigitalServoDriver()
print(DigitalDriver.port_name,DigitalDriver.port_list)

with serial.Serial(DigitalDriver.port_name[1], DigitalDriver.baud_rate, timeout=None) as ser:
    # 启动
    start = [0x00, 0x00, 0x01, 0x01]
    ser.write(bytes(start))
    # PC数字输入模式
    pc = [0x02, 0x00, 0xc4, 0xc6]
    ser.write(bytes(pc))
    #   PC数字输入
    speed = [0x06, 0x00, 0x88, 0x8e]
    ser.write([0x06, 0x00, 0x88, 0x8e])
    # 等待60秒
    while True:
        time.sleep(0.5)
        watch = [0x80, 0x00, 0x80]
        ser.write(bytes(watch))
    # 关闭电机
    ser.write(0x00)
    ser.write(0x00)
    ser.write(0x00)
    ser.write(0x00)
