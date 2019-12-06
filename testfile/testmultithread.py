#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testmultithread.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/17 23:00   msliu      1.0      

@Description
------------
None
"""

import threading
import time
import DigitalDriver.ControlDriver as CD


def loop(control):
    while True:
        time.sleep(5)
        temp = input("type: a->speed b->omega c->r clear")
        if temp == "a":
            speed = input("speed")
            control.speed = float(speed)
        elif temp == "b":
            omega = input("omega")
            control.omega = float(omega)
        elif temp == "c":
            r = input("radius")
            control.radius = float(r)
        elif temp == "clear":
            control.omega=0
            control.speed=0
            control.radius = 0

        if temp == -1:
            control.flag_end = 1
            break


if __name__ == '__main__':
    # t = threading.Thread(target=loop(name))
    cd = CD.ControlDriver()
    p1 = threading.Thread(target=loop, args=(cd,))
    p2 = threading.Thread(target=cd.control_part, args=())
    print("hehe")
    p2.start()
    p1.start()
