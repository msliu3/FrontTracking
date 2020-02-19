#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testDeepLearningControl.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/2/17 21:53   msliu      1.0      

@Description
------------
None
"""

import os
import sys
import time
import numpy as np

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

import FootDetector.DemonstrationProcess as DP
import Control.DeepLearning_DetectCase as DL
import threading
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC
import LegDetector.LegInformation as LegLidar
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


def loop2(deep, queue, leg):
    sum = 0
    ir_data = []
    lidar_data = []
    while True:
        time.sleep(1)
        while len(ir_data) <= deep.sample_num:

            if len(ir_data) == deep.sample_num:
                ir_data.pop(0)
                lidar_data.pop(0)
            if not queue.empty():
                # print("into queue")
                temps = queue.get(block=False)
                ir_data.append(np.array(temps).reshape([1, 24 * 32]))
                print("leg:", leg.left_leg_x, leg.left_leg_y, leg.right_leg_x, leg.right_leg_y)
                leg_temp = np.array([leg.left_leg_x, leg.left_leg_y, leg.right_leg_x, leg.right_leg_y]).reshape([1, 4])
                lidar_data.append(leg_temp)
            if len(ir_data) == deep.sample_num:
                break

        ir_np = np.array(ir_data).reshape([32 * 24, deep.sample_num])
        leg_np = np.array(lidar_data).reshape([4, deep.sample_num])
        sum += 1
        print(leg_np[0])
        # print("times: ",sum)
        # deep.print_predict_result(ir_np, leg_np)


if __name__ == '__main__':
    # 开启IR camera，开启一条进程处理数据
    queue = multiprocessing.Queue()
    pd = DP.DemonProcess()
    leg = LegLidar.LegInformation()
    thread_start = threading.Thread(target=leg.loop, args=())
    thread_ir = multiprocessing.Process(target=pd.start_Demon_for_DL, args=(queue,))
    deep = DL.DeepLearningDetectCase(model_name="DNN1.ckpt")
    p2 = threading.Thread(target=loop2, args=(deep, queue, leg))
    thread_start.start()

    # cd = CD.ControlDriver()
    # event = threading.Event()
    # thread_control_driver = threading.Thread(target=cd.control_part, args=())
    # thread_control_driver.start()
    # p1 = threading.Thread(target=loop, args=(cd, mc, pc, event))

    thread_ir.start()
    p2.start()
    # p1.start()
