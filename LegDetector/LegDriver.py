#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
"""
@File    :   LegDriver.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/15 21:55   msliu      1.0         None
"""

import threading
import DigitalDriver.ControlDriver as CD
import LegDetector.Line_control as Line_control
import time


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
    # while True:
    #     time.sleep(5)
    #     temp = str(input("type: a->speed b->omega c->r clear"))
    #     if temp == "a":
    #         speed = input("speed")
    #         control.speed = float(speed)
    #     elif temp == "b":
    #         omega = input("omega")
    #         control.omega = float(omega)
    #     elif temp == "c":
    #         r = input("radius")
    #         control.radius = float(r)
    #     elif temp == "clear":
    #         control.omega = 0
    #         control.speed = 0
    #         control.radius = 0
    #
    #     if temp == -1:
    #         control.flag_end = 1
    #         break
    pass


cd = CD.ControlDriver()
thread1 = threading.Thread(target=cd.control_part, args=())
thread1.start()
line = Line_control.LineDemo()
thread2 = threading.Thread(target=loop, args=(cd,))
thread2.start()
