#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import os
import sys
import time

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
import threading
import DigitalDriver.ControlandOdometryDriver as CD
import Control.PositionControl2 as PC



def loop(event, control, ii):
    while True:
        time.sleep(1)
        end = input("end?\n")
        if end == "Y":
            print("------------------------------------------------")
            ii.speed = 0.0
            ii.radius = 0.0
            ii.omega = 0.0
            ii.set_driver(control)
            event.set()
        else:
            default_value = 0.15
            status = input("Forward (F) or Left(L) or Right(R)?\n")
            if status == "F":
                ii.speed = default_value
                ii.radius = 0.0
                ii.omega = 0.0
                ii.set_driver(control)
            elif status == "L":
                ii.radius = float(input("input Radius:\n"))
                ii.speed = 0.0
                ii.omega = default_value
                ii.set_driver(control)
                print(ii.radius)
            elif status == "R":
                ii.radius = float(input("input Radius:\n"))
                ii.speed = 0.0
                ii.omega = - default_value
                ii.set_driver(control)
                print(ii.radius)
            else:
                pass


class input_information(object):
    def __init__(self):
        self.x = 0.0
        self.theta = 0.0
        self.speed = 0.2
        self.omega = 0.0
        self.radius = 0.0
        pass

    def set_driver(self, control_driver):
        control_driver.speed = self.speed
        control_driver.radius = self.radius
        control_driver.omega = self.omega
        # time.sleep(times)
        return

if __name__ == '__main__':

    cd = CD.ControlDriver()

    event = threading.Event()
    ii = input_information()

    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    loop_1 = threading.Thread(target=loop, args=(event, cd, ii,))
    loop_1.start()

