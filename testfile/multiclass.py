#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   multiclass.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/22 20:47   msliu      1.0      

@Description
------------
None
"""
from multiprocessing import Process
import time


class multiprocess(Process):
    def __init__(self, name):
        Process.__init__(self)
        self.name = name

    def func1(self):
        while True:
            print(self.name)
            time.sleep(5)
            print("end_func1")

    def run(self):
        self.func1()

    pass


if __name__ == '__main__':
    ml = multiprocess("hehe")
    ml.start()
