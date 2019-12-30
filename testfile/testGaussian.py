#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testGaussian.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/30 18:38   msliu      1.0      

@Description
------------
None
"""

"""
绘制正太分布函数曲线图

"""
import matplotlib.pyplot as plt
import math
import numpy as np
import matplotlib

matplotlib.rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题
plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体

x = np.linspace(-10, 30, num=1000)  # x轴的取值范围
std1 = 1/math.sqrt(2*math.pi)  # 定义标准差, 并输入标准差
mean1 = 0  # 定义均值,并输入均值
fx1 = 1 / (std1 * pow(2 * math.pi, 0.5)) * np.exp(-((x - mean1) ** 2) / (2 * std1 ** 2))  # 概率密度函数公式

if fx1>0.2:
    print(x)

std2 = 2
mean2 = 10
fx2 = 1 / (std2 * pow(2 * math.pi, 0.5)) * np.exp(-((x - mean2) ** 2) / (2 * std2 ** 2))  # 概率密度函数公式

std3 = 4
mean3 = 10
fx3 = 1 / (std3 * pow(2 * math.pi, 0.5)) * np.exp(-((x - mean3) ** 2) / (2 * std3 ** 2))  # 概率密度函数公式

std4 = 8
mean4 = 10
fx4 = 1 / (std4 * pow(2 * math.pi, 0.5)) * np.exp(-((x - mean4) ** 2) / (2 * std4 ** 2))  # 概率密度函数公式

# 多条曲线在同一张图上进行对比
plt.plot(x, fx1, label='std1 = 1')  # 绘制概率密度函数图像
plt.plot(x, fx2, label='std2 = 2')
plt.plot(x, fx3, label='std3 = 4')
plt.plot(x, fx4, label='std4 = 8')
plt.legend()  # 显示标签 label
plt.xlabel("数值")
plt.ylabel('数值的概率')
plt.title('服从正太分布的概率密度图')
plt.show()  # 显示图像
