#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testserial.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/1/18 4:10   msliu      1.0      

@Description
------------
None
"""

# import lib
import serial

ser_l = serial.Serial("COM14", 57600, timeout=1)
start = [0x00, 0x00, 0x01, 0x01]
ser_l.write(bytes(start))
list_data = []
# for i in range(2):
#     if ser_l.readable():
#         data = ser_l.read()
#         print(data)
#         list_data.append(data)
#     else:
#         print("no read")
# print(list_data)
try:
    ser_l.read(2)
finally:
    print("no read data")
