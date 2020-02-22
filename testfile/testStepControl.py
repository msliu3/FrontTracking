#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testStepControl.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/29 20:50   msliu      1.0      

@Description
------------
None
"""

# import lib
import threading
import time
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC


def loop(control, pc):
    while True:
        time.sleep(5)
        temp = input("assign")
        if temp == "a":
            x = input("expect_x")
            theta = input("theta")
            pc.set_expect(float(x), float(theta))
            # pc.action_forward_and_turning(control)
            pc.action_forward_back(control)
        if temp == "end":
            print("end!!!")
            control.flag_end = 1
            break


if __name__ == '__main__':
    pc = PC.PositionControl()
    cd = CD.ControlDriver()
    p1 = threading.Thread(target=loop, args=(cd,pc))
    p2 = threading.Thread(target=cd.control_part, args=())
    p2.start()
    p1.start()
