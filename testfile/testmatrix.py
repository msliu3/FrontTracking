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
import ProcessFunc as pf

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
