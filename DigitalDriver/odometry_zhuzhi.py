'''
@File    :   odometry.py
@Contact :   mspzhuz@gmail.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/11 22:21   zzhu       1.0

@Description
---------------------------------
接受两轮编码器的位置信息，计算walker相对于上一时刻的偏转角dθ，坐标变化dx、 dy
'''

import numpy as np
import matplotlib.pyplot as plt
from robotserialcontrol import ControlDriver as CD
import math

class Odometry:
    def __init__(self, X=0.0, Y=0.0, THETA=0.0, Odo_l = 0, Odo_r = 0, plot = False):
        self.Odo_l, self.Odo_r = Odo_l, Odo_r
        self.d_theta = 0.0
        self.d_l, self.d_r = 0.0, 0.0
        self.p_l, self.p_r = 0, 0
        self.Radius = 0.0
        self.X, self.Y = X, Y
        self.dX, self.dY = 0.0, 0.0
        self.plot = plot
        self.THETA = THETA

    #更新里程计读取到的信息
    def updatePose(self, Odo_l, Odo_r):
        self.Odo_l, self.Odo_r = Odo_l, Odo_r
        #计算两轮相对于上一时刻的位移
        self.d_l = (self.Odo_l - self.p_l) / 4096 * 2 * math.pi * 0.085
        self.d_r = (self.Odo_r - self.p_r) / 4096 * 2 * math.pi * 0.085
        #保存此时刻编码器数据
        self.p_l = self.Odo_l
        self.p_r = self.Odo_r

        # 计算dθ，逆时针为正，顺时针为负
        self.d_theta = (self.d_r - self.d_l) / 0.54  # 左转>0, 右转<0
        # 更新朝向角θ（θ∈(-π，π]）
        self.THETA += self.d_theta
        if self.THETA > math.pi:
            self.THETA -= 2*math.pi
        elif self.THETA <= -math.pi:
            self.THETA += 2*math.pi

        # 计算转弯半径 R
        if self.d_theta:
            if (self.d_l + self.d_r) == 0:  # 原地转向或静止
                self.Radius = 0
            else:
                if self.d_l * self.d_r > 0:  # 转向中心在walker之外
                    self.Radius = min(abs(self.d_l / self.d_theta), abs(self.d_r / self.d_theta)) + 0.27
                else:  # 转向中心在walker之内
                    self.Radius = 0.27 - min(abs(self.d_l / self.d_theta), abs(self.d_r / self.d_theta))


        # 计算坐标变化dX, dY
        dx, dy = 0.0, 0.0   # Walker坐标系下的坐标变化
        if (self.d_l + self.d_r)==0: # 原地转向或静止
            self.dX, self.dY = 0.0, 0.0
        elif self.d_l > self.d_r and self.d_l > 0:  # 右前
            dx = self.Radius * (1-math.cos(self.d_theta))
            dy = self.Radius * math.sin(self.d_theta)
        elif self.d_l > self.d_r and self.d_l <= 0: # 右后
            dx = self.Radius * (math.cos(self.d_theta)-1)
            dy = -self.Radius * math.sin(self.d_theta)
        elif self.d_r > self.d_l and self.d_r > 0:  # 左前
            dx = self.Radius * (math.cos(self.d_theta)-1)
            dy = self.Radius * math.sin(self.d_theta)
        elif self.d_r > self.d_l and self.d_r <= 0: # 左后
            dx = self.Radius * (math.cos(self.d_theta) - 1)
            dy = -self.Radius * math.sin(self.d_theta)

        #Walker坐标系下的坐标变化dx,dy → 绝对坐标系下的坐标变化 dX, dY
        # dX = cosθ·dx - sinθ·dy,  dY = sinθ·dx + cosθ·dy （参考复平面的2D旋转公式）
        self.dX = dx * math.cos(self.THETA) - dy * math.sin(self.THETA)
        self.dY = dx * math.sin(self.THETA) + dy * math.cos(self.THETA)
        #更新绝对坐标系下坐标变化
        self.X += self.dX
        self.Y += self.dY

        return (self.X, self.Y, self.THETA)

    def getOriChange(self):
        return self.d_theta  # 弧度制

    def getTurningRadius(self):
        return self.Radius

if __name__ == "__main__":

    cd = CD.ControlDriver()
    cd.start()