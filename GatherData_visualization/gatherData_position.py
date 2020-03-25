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


import FootDetector.DemonstrationProcess as DP
import LegDetector.LegInformation as LegLidar
import DigitalDriver.ControlandOdometryDriver as CD
import SoftSkin.SoftSkin as SS
import threading
import multiprocessing


def loop(event):
    while True:
        time.sleep(1)
        end = input("end?")
        if end == "end":
            print("------------------------------------------------")
            event.set()


if __name__ == '__main__':
    pd = DP.DemonProcess()
    cd = CD.ControlDriver(record_mode=True)
    ss = SS.SoftSkin()
    # IMU = imu.ArduinoRead()
    # queue = multiprocessing.Queue()
    # thread_imu = threading.Thread(target=IMU.reading_data_from_arduino, args=())
    # thread_imu.start()
    event = threading.Event()
    thread_ir = multiprocessing.Process(target=pd.start_Demon, args=(None, True))
    thread_ir.start()
    thread_soft_skin = threading.Thread(target=ss.label_front_follow, args=())
    thread_soft_skin.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    leg = LegLidar.LegInformation()
    thread_start = threading.Thread(target=leg.loop, args=())
    thread_start.start()
    event.clear()
    p1 = threading.Thread(target=loop, args=(event,))
    p1.start()

    data_path = father_path + os.path.sep + "resource" + os.path.sep + "leg_odo.txt"
    print(data_path)
    with open(data_path, 'w') as file:
        while True:
            time.sleep(0.25)
            # print("%f"%cd.odo.Radius)
            print("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%s\t%f" % (
                time.time(),
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
                ss.label,
                cd.odo.getTurningRadius()
                # IMU.imu_human
            ))
            # file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%s\t%f\n" % (time.time(),
            #                                                              cd.odo.getROS_XYTHETA()[0],
            #                                                              cd.odo.getROS_XYTHETA()[1],
            #                                                              cd.odo.THETA,
            #                                                              # cd.odo.getROS_XTTHETA()[2],
            #                                                              cd.odo.get_dxdydtheta()[0],
            #                                                              cd.odo.get_dxdydtheta()[1],
            #                                                              leg.left_leg_x,
            #                                                              leg.left_leg_y,
            #                                                              leg.right_leg_x,
            #                                                              leg.right_leg_y,
            #                                                              ss.label,
            #                                                              cd.odo.Radius
            #                                                              ))
            file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%s\t%f\t%f\t%f\t%f\n" % (time.time(),
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
                                                                             ss.label,
                                                                             cd.odo.Radius,
                                                                             cd.odo.d_l,
                                                                             cd.odo.d_r,
                                                                             cd.odo.d_theta
                                                                             ))
            # file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (time.time(),
            #                                                              cd.odo.getROS_XYTHETA()[0],
            #                                                              cd.odo.getROS_XYTHETA()[1],
            #                                                              cd.odo.THETA,
            #                                                              # cd.odo.getROS_XTTHETA()[2],
            #                                                              cd.odo.get_dxdydtheta()[0],
            #                                                              cd.odo.get_dxdydtheta()[1],
            #                                                              leg.left_leg_x,
            #                                                              leg.left_leg_y,
            #                                                              leg.right_leg_x,
            #                                                              leg.right_leg_y,
            #
            #                                                              ))
            # file.write("sasasasasasa \n")
            if event.is_set():
                break
        file.close()
        # thread_control_driver.join()
        # thread_start.join()
        # p1.join()
