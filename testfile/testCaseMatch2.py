#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testStepControl.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/1/3 2:11      msliu      1.0

@Description
------------
None
"""
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/FootDetector"))
sys.path.append(BASE_DIR)
BASE_DIR1 = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/Control"))
sys.path.append(BASE_DIR1)
BASE_DIR2 = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/DigitalDriver"))
sys.path.append(BASE_DIR1)

import Control.MatchCase as MC
import FootDetector.DemonstrationProcess as DP
# import numpy as np

import threading
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC


def loop(control, matcher, pc, e):
    while True:
        e.wait()
        if pc.action_over:
            if matcher.back or matcher.forward:
                pc.action_forward_back(control)
            elif matcher.turning:
                pc.action_forward_and_turning(control)

def loop2(matcher):
    while True:
        matcher.detect_front_and_back_foot()
        x, theta = matcher.detect_case()
        # print("x and theta", x, theta)
        pc.set_expect(x, theta)
        event.set()


if __name__ == '__main__':
    pd = DP.DemonProcess()
    cd = CD.ControlDriver()
    mc = MC.MatchCase(foot=pd.foot)
    pc = PC.PositionControl()
    event = threading.Event()
    thread_ir = threading.Thread(target=pd.start_Demon, args=())
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_ir.start()
    thread_control_driver.start()
    p1 = threading.Thread(target=loop, args=(cd, mc, pc, event))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(mc,))
    p2.start()
