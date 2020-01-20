#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testStepControl.py
@Contact :   liumingshanneo@163.com

@Modify   Time      @Author    @Version
--------  ----      -------    --------
2019/1/18 8:11      msliu      1.0

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

import FootDetector.DemonstrationProcess as DP
import LegDetector.LegInformation as LegLidar
import GatherData.gatherIMU as imu
# import numpy as np

import threading
import DigitalDriver.ControlDriver_zhuzhi as CD
import multiprocessing


def loop(event):
    while True:
        time.sleep(1)
        end = input("end?")
        if end == "end":
            print("------------------------------------------------")
            event.set()



def loop2(matcher, queue, event):
    while True:
        time.sleep(0.25)
        # print(queue.qsize())
        if not queue.empty():
            print("assign")
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
    IMU = imu.ArduinoRead()
    event = threading.Event()
    # queue = multiprocessing.Queue()
    thread_ir = multiprocessing.Process(target=pd.start_Demon, args=(queue,))
    thread_ir.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    leg = LegLidar.LegInformation()
    thread_start = threading.Thread(target=leg.loop, args=())
    thread_start.start()
    thread_imu = threading.Thread(target=IMU.reading_data_from_arduino,args=())
    thread_imu.start()
    event.clear()
    p1 = threading.Thread(target=loop, args=(event,))
    p1.start()

    data_path = father_path + os.path.sep + "resource" + os.path.sep + "leg_odo.txt"
    print(data_path)
    with open(data_path, 'w') as file:
        while True:
            time.sleep(0.25)
            print(cd.odo.getROS_XYTHETA()[2])
            print("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f" % (time.time(),
                                                          cd.odo.getROS_XYTHETA()[0],
                                                          cd.odo.getROS_XYTHETA()[1],
                                                            cd.odo.THETA,
                                                          # cd.odo.getROS_XTTHETA()[2],
                                                          cd.odo.get_dxdydtheta()[0],
                                                          cd.odo.get_dxdydtheta()[1],
                                                          leg.left_leg_x,
                                                          leg.left_leg_y,
                                                          leg.right_leg_x,
                                                          leg.right_leg_y,
                                                            IMU.imu_human
                                                          ))
            file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (time.time(),
                                                                 cd.odo.getROS_XYTHETA()[0],
                                                                 cd.odo.getROS_XYTHETA()[1],
                                                                 cd.odo.THETA,
                                                                 # cd.odo.getROS_XTTHETA()[2],
                                                                 cd.odo.get_dxdydtheta()[0],
                                                                 cd.odo.get_dxdydtheta()[1],
                                                                 leg.left_leg_x,
                                                                 leg.left_leg_y,
                                                                 leg.right_leg_x,
                                                                 leg.right_leg_y,
                                                                 IMU.imu_human
                                                                 ))
            if event.is_set():
                break
        file.close()
        # thread_control_driver.join()
        # thread_start.join()
        # p1.join()
