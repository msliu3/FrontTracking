#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   plot_odom_IMU.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/1/15 5:29   msliu      1.0      

@Description
------------
None
"""

from matplotlib import pyplot as plt
import os
import datetime
import pandas as pd
import re


def read_file_to_list(file):
    list_data = []
    with open(file) as f:
        line = f.readline()
        line = line.rstrip('\n')
        list_data.append(float(line))
        while line:
            line = f.readline()
            line = line.rstrip('\n')
            if line != "":
                list_data.append(float(line))
    return list_data


def difference_list(list_data):
    dif_list = []
    for i in range(1, len(list_data) - 1):
        if list_data[i] * list_data[i - 1] < 0 and abs(list_data[i]) + abs(list_data[i - 1]) > 180:
            # print("into", list_data[i-1], list_data[i],end=" ")
            dif = (list_data[i-1] / abs(list_data[i-1])) * 180 - list_data[i - 1]
            dif += list_data[i] - (list_data[i] / abs(list_data[i])) * 180
            # print(dif)
        else:
            dif = list_data[i] - list_data[i - 1]
        dif_list.append(dif)
    return dif_list


def strToFloat(number):
    try:
        return float(number)
    except:
        return number


if __name__ == '__main__':
    IMU_file = "../resource/1-16IMU.txt"
    Odo_file = "../resource/1-6Odom.txt"
    # time_data = "04:23:23.232"
    # print(re.findall(r"(\d{2}\:|\d{2}\.\d+)",time_data))
    odo_data = read_file_to_list(Odo_file)
    print(odo_data)
    dif_odo = difference_list(odo_data)
    # for item in dif_odo:
    #     print(item)
    imu_data = read_file_to_list(IMU_file)
    print(imu_data)
    dif_imu = difference_list(imu_data)
    #
    # plt.plot(dif_odo)
    # # plt.plot(dif_imu)
    plt.plot(odo_data, color="r")
    plt.plot(imu_data, color="b")
    plt.show()

    # listFromLine = list(map(strToFloat, imu))
    # print(listFromLine)
