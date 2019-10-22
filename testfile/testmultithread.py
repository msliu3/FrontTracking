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
import multiprocessing as ml
import robotserialcontrol.ControlDriver as CD


def loop(control):
    while True:
        time.sleep(15)
        print("end_this")
        # temp = input("num")
        # if temp == -1:
        #     control.flag_end = 1
        #     break
        # control.speed = temp


if __name__ == '__main__':
    # t = threading.Thread(target=loop(name))
    cd = CD.ControlDriver()
    p1 = ml.Process(target=loop, args=(cd,))
    p2 = ml.Process(target=CD.ControlDriver.control_part, args=())
    print("hehe")
    p2.start()
    p1.start()
