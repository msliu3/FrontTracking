#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   MatchCase.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/01/02 02:02   msliu      1.0

@Description
------------

用过FootInformation 和 LegInformation

"""

import os, sys

BASE_DIR = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/FootDetector"))
sys.path.append(BASE_DIR)
BASE_DIR1 = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/Control"))
sys.path.append(BASE_DIR1)
BASE_DIR2 = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/DigitalDriver"))
sys.path.append(BASE_DIR1)

import Control.MatchCase as MC
import FootDetector.DemonstrationProcess as DP
import FootDetector.ProcessFunc as pf
import numpy as np

import threading
import time
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC


def loop(control, pc, e, matcher):
    while True:
        e.wait()
        if pc.action_over:
            if matcher.back or matcher.forward:
                pc.action_forward_back(control)
            elif matcher.turning:
                pc.action_forward_and_turning(control)


if __name__ == '__main__':
    event = threading.Event()
    pc = PC.PositionControl()
    cd = CD.ControlDriver()
    dp = DP.DemonProcess()
    matcher = MC.MatchCase(dp.foot)
    p1 = threading.Thread(target=loop, args=(cd, pc, event, matcher))
    p1.start()
    p2 = threading.Thread(target=cd.control_part, args=())
    p2.start()
    head = []
    data = []
    filter_data = []
    rest_num = 5
    while True:
        s = dp.serial.read(1).hex()
        if s != "":
            s = int(s, 16)
        head.append(s)

        if len(head) == dp.head_size:
            if dp.check_head_data(head):
                temp = dp.serial.read(1540)
                data.append(temp.hex())
                head.clear()
            else:
                head.pop(0)
            # 将读到的数据进行展示
            if len(data) == rest_num:
                temp, ir_np, foot = dp.demonstrate_data(data[rest_num - 1], filter_data,
                                                        filter_num=2)  # ,zoom_filter=Image.HAMMING
                # ir_np = pf.draw_hist(ir_np)
                if foot:

                    # ir_np = pf.image_processing_mean_filter(ir_np, kernel_num=32)
                    # pf.show_temperature(temp)
                    # ir_np = pf.image_processing_contrast_brightness(ir_np, 1.6, -0.8)
                    ir_np, contours = dp.binary_image(np.array(ir_np))
                    dp.find_foot_ankle(ir_np, contours)

                    matcher.detect_front_and_back_foot()
                    x, theta = matcher.detect_case()
                    # print("x and theta", x, theta)
                    pc.set_expect(x, theta)
                    event.set()
                    # matcher.img_detect_front_foot()
                    # matcher.distance_detect_front_foot()

                    if dp.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
                        break

                # ir_np, contours = dp.binary_image(np.array(ir_np))
                # dp.find_foot_ankle(ir_np, contours)
                # if dp.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
                #     break
                data.pop(rest_num - 1)
                data.pop(0)
                # data.pop(0)
