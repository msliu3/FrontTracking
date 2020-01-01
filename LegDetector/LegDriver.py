#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
@File    :   LegDriver.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/15 21:55   msliu      1.0         None
"""

import os, sys
import threading

BASE_DIR = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/DigitalDriver"))
print(BASE_DIR+"/..")
sys.path.append(BASE_DIR)
from DigitalDriver import ControlDriver as CD
import LegDetector.Line_control as Line_control


# cd = CD.ControlDriver()
# thread1 = threading.Thread(target=cd.control_part, args=())
# thread1.start()
# line = Line_control.LineDemo()
# while True:
#     cd.speed = line.control_system(-10)
#     # print(cd.speed)

def loop(control):
    while True:
        control.speed = line.control_system(0.37)
        # print(cd.speed)

    pass


cd = CD.ControlDriver()
thread1 = threading.Thread(target=cd.control_part, args=())
thread1.start()
line = Line_control.LineDemo()
thread2 = threading.Thread(target=loop, args=(cd,))
thread2.start()
