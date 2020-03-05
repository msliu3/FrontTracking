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
import threading
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import Control.MatchCase2 as MC
import FootDetector.DemonstrationProcess as DP
# import numpy as np

import DigitalDriver.ControlandOdometryDriver as CD
import Control.PositionControl2 as PC
import multiprocessing


def loop(control, pc):
    while True:
        time.sleep(0.5)
        pc.top_decision(control)


def loop2(matcher, queue, pc):
    state_list = []
    state_num = 5
    while True:
        time.sleep(0.3)
        # print(queue.qsize())
        if not queue.empty():
            # print("assign")
            matcher.foot = queue.get(block=False)
        matcher.detect_front_and_back_foot()
        x, theta = matcher.detect_case()
        state_list.append(matcher.state)
        if len(state_list) == state_num:
            if state_list.count(state_list[-1]) == state_num:
                print("action", x, theta)
                pc.set_expect(x, theta)
                # print("data read!!!!!!")
                matcher.clear_expect()
                matcher.clear_foot_and_leg()
        # for i in state_list:
        #     print(i,end=" ")
        # print()
        if len(state_list) >= state_num:
            state_list.pop(0)


def loop_stop():
    global flag_stop
    end = input()
    while end == "":
        flag_stop = 0


if __name__ == '__main__':
    pd = DP.DemonProcess()
    cd = CD.ControlDriver()
    mc = MC.MatchCase(foot=pd.foot)
    pc = PC.PositionControl2()
    event = threading.Event()
    queue = multiprocessing.Queue()

    flag_stop = 1

    thread_ir = multiprocessing.Process(target=pd.start_Demon, args=(queue,))
    thread_ir.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()
    p1 = threading.Thread(target=loop, args=(cd, pc))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(mc, queue, pc))
    p2.start()
    p3 = threading.Thread(target=loop_stop(), args=())
    p3.start()



