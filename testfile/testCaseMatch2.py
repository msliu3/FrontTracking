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
import time

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import threading

import Control.MatchCase2 as MC
import FootDetector.DemonstrationProcess as DP
# import numpy as np

import threading
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC
import multiprocessing


def loop(control, matcher, pc, e):
    while True:
        e.wait()
        e.clear()
        if pc.action_over:
            if matcher.back or matcher.forward:
                pc.action_forward_back(control)
                matcher.clear_case()
            elif matcher.turning:
                pc.action_forward_and_turning(control)
                matcher.clear_case()


def loop2(matcher, queue, event):
    while True:
        time.sleep(0.3)
        # print(queue.qsize())
        if not queue.empty():
            # print("assign")
            matcher.foot = queue.get(block=False)
        matcher.detect_front_and_back_foot()
        x, theta = matcher.detect_case()
        # print("x and theta matcher", x, theta)
        pc.set_expect(x, theta)
        event.set()
        matcher.clear_expect()
        matcher.clear_foot_and_leg()



if __name__ == '__main__':
    pd = DP.DemonProcess()
    cd = CD.ControlDriver()
    mc = MC.MatchCase(foot=pd.foot)
    pc = PC.PositionControl()
    event = threading.Event()
    queue = multiprocessing.Queue()
    thread_ir = multiprocessing.Process(target=pd.start_Demon, args=(queue,))
    thread_ir.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    p1 = threading.Thread(target=loop, args=(cd, mc, pc, event))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(mc, queue, event))
    p2.start()
