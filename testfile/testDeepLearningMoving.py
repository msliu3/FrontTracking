#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testDeepLearningMoving.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/3/25 16:50   msliu      1.0      

@Description
------------
测试DeepLearning和Control结合
这里用的是Chongyu的新控制逻辑
"""

# import lib
import os
import sys
import time
import numpy as np

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

import FootDetector.DemonstrationProcess as DP
import Control.DeepLearning_DetectCase as DL
import Control.MatchCase_NN as MC
import threading
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC
import LegDetector.LegInformation as LegLidar
import multiprocessing


def control_loop(position_control, control_driver, flag, event):
    while True:
        event.wait()
        if position_control.action_over:
            if matcher.back or matcher.forward:
                # print("back_forward", pc.expect_x, pc.expect_theta)
                position_control.action_forward_back(control_driver)
                matcher.clear_case()
            elif matcher.turning:
                # print("turn")
                position_control.action_forward_and_turning(control_driver)
                matcher.clear_case()
        event.clear()


def loop2(deep, queue, matcher, pc, event,control_driver):
    while True:
        time.sleep(0.05)
        result = 1
        if not queue.empty():
            # print("into queue")
            # matcher.clear_foot_and_leg()
            temps = queue.get(block=False)
            # print("leg:", matcher.leg.left_leg_x, matcher.leg.left_leg_y, matcher.leg.right_leg_x,
            #       matcher.leg.right_leg_y)
            leg_temp = np.array([matcher.leg.left_leg_x, matcher.leg.left_leg_y, matcher.leg.right_leg_x,
                                 matcher.leg.right_leg_y]).reshape([1, 4])

            ir_np = np.array(temps).reshape([32 * 24, deep.sample_num])
            leg_np = np.array(leg_temp).reshape([4, deep.sample_num])
            # print(leg_np[0])
            # print("times: ",sum)
            result = deep.print_predict_result(ir_np, leg_np)
            matcher.detect_front_and_back_foot()
            x, theta = matcher.detect_case(result)
            print(x, theta)
            pc.set_expect(x, theta)
            event.set()
            matcher.clear_expect()
        # matcher.clear_foot_and_leg()


if __name__ == '__main__':
    # 开启IR camera，开启一条进程处理数据
    queue = multiprocessing.Queue()
    pd = DP.DemonProcess()
    matcher = MC.MatchCase(foot=pd.foot)
    position_control = PC.PositionControl()
    event = threading.Event()
    nn_model = DL.DeepLearningDetectCase(model_name="dnn1.ckpt")
    cd = CD.ControlDriver(left_right=1)
    flag_stop = 1

    thread_ir = multiprocessing.Process(target=pd.start_Demon_for_DL, args=(queue,))
    thread_ir.start()
    p1 = threading.Thread(target=control_loop, args=(position_control, cd, flag_stop, event))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(nn_model, queue, matcher, position_control, event,cd))  # nn_model,
    p2.start()

    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()
