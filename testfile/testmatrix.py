#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testmatrix.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/30 22:10   msliu      1.0      

@Description
------------
None
"""

import numpy as np
from FootDetector import ProcessFunc as pf
import math

a = np.array([[1, 2], [3, 4], [5, 6]])
b = np.array([[1, 2], [3, 4], [5, 6]])
c = np.array([[1, 2], [3, 4], [5, 6]])
d = np.array([[1, 2], [3, 4], [5, 6]])
temp = [a, b, c, d]
print(a.shape)
sum_np = np.array(np.zeros(a.shape))
print(sum_np)
for item in temp:
    sum_np += item
print(sum_np / len(temp))
list_ave = pf.filter_for_ir(temp)
print(list_ave.tolist()[0])

print("-------------------------------")
test1 = dict()
test2 = {"a": 1, "b": 2}
print(test2,test1)
test1 = test2
print(test1)
print("-------------------------------")
# original_x = data.pose.position.x - self.robot_x
# original_y = data.pose.position.y - self.robot_y
original_x = 0.5
original_y = 0
theta = math.radians(-90)
print(theta,math.radians(-90))

trans = np.array([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]])
result = np.dot(trans, np.array([[original_x], [original_y]]))
print(result[0],result[1])