#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   multiProcess.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/22 16:16   msliu      1.0      

@Description
------------
None
"""

import multiprocessing
import time


def worker_1(interval):
    while True:
        print("worker_1")
        time.sleep(interval)
        print("end_worker_1")


def worker_2(sed):
    print("worker_2")
    time.sleep(sed)
    print("end_worker_2")


if __name__ == '__main__':
    p1 = multiprocessing.Process(worker_1(2))
    p2 = multiprocessing.Process(worker_2(3))

    # p1 = multiprocessing.Process(target=worker_1, args=(2,))
    # p2 = multiprocessing.Process(target=worker_2, args=(3,))
