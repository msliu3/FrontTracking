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

# import lib
import serial
ser = serial.Serial("COM15",baudrate=115200,timeout=None)
while True:
    l = ser.readall()
    print(l.hex())
