#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   Line_control.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/11/28 11:49   msliu      1.0

@Description
------------
None
"""
import threading
from threading import Thread
import LegDetector.LegInformation as LegLidar

class LineDemo(Thread):
    def __init__(self):
        self.leg = LegLidar.LegInformation()
        # start read data
        thread_start = threading.Thread(target=self.leg.loop, args=())
        thread_start.start()
        self.base_line = 0.0
        pass

    def obtain_leg_information(self):
        """
        kalman filter
        """

        pass

    def control_system(self, kp=0.016):
        temp_l = self.leg.left_leg_x
        temp_r = self.leg.right_leg_x
        # print("left_x : "+ str(temp_l)+" right_x : "+str(temp_r))
        if temp_l > temp_r:
            error = self.base_line - temp_l
        else:
            error = self.base_line - temp_r
        print("error:" + str(error))
        return error * kp


if __name__ == '__main__':
    line = LineDemo()
    while True:
        result = line.control_system()
        # print(result)
