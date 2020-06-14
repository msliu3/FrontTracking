import numpy as np
import matplotlib.pyplot as plt
from DigitalDriver import ControlDriver as CD
import time
import math


class Odometry:
    def __init__(self, X=0.0, Y=0.0, THETA=0.0, Odo_l=0, Odo_r=0,
                 imu_yaw=0.0, tick_threshold=0):
        self.Odo_l, self.Odo_r = Odo_l, Odo_r
        self.d_theta = 0.0
        self.d_l, self.d_r = 0.0, 0.0
        self._p_l, self._p_r = 0, 0
        self.v_l, self.v_r = 0.0, 0.0  # m/s
        self.V, self.OMEGA = 0.0, 0.0
        self.Radius = 0.0
        self._X, self._Y = X, Y
        self._dX, self._dY = 0.0, 0.0
        self.dx, self.dy = 0.0, 0.0
        self._THETA = THETA
        self.imu_yaw = imu_yaw
        self.tick_threshold = tick_threshold
        self._previous_time = time.time()
        self._wheel_base = 0.54
        # print('X=', self.X, 'm;  Y=', self.Y, 'm;  THETA=', self.THETA / math.pi * 180, '°')

    # 更新里程计读取到的信息
    # Update Odometry message
    def updatePose(self, *args):
        currnt_time = time.time()
        dt = currnt_time - self._previous_time
        self.Odo_l, self.Odo_r = args[0], args[1]
        if len(args) > 2:
            self.imu_yaw = args[-1]
        # print("Digital distance:",self.Odo_l,self.Odo_r)

        # 计算两轮相对于上一时刻的位移
        # Calculate the wheel displacement dl, dr
        if abs(self.Odo_l - self._p_l) >= self.tick_threshold:
            self.d_l = ((self.Odo_l - self._p_l) / 4096) * 2 * math.pi * 0.085
        else:
            self.d_l = 0
        if abs(self.Odo_r - self._p_r) >= self.tick_threshold:
            self.d_r = ((self.Odo_r - self._p_r) / 4096) * 2 * math.pi * 0.085
        else:
            self.d_r = 0
        self.v_l = self.d_l / dt
        self.v_r = self.d_r / dt
        # print('Left displacement: ', self.d_l, 'm;  Right displacement: ', self.d_r, 'm;')

        # 保存此时刻编码器数据
        # Save encoder position of this step
        self._p_l = self.Odo_l
        self._p_r = self.Odo_r

        # 计算dθ，逆时针为正，顺时针为负
        # Calculate dθ and OMEGA, counterclockwise+, clockwise-
        self.d_theta = (self.d_r - self.d_l) / self._wheel_base  # 左转>0, 右转<0
        self.OMEGA = self.d_theta / dt

        # 计算转弯半径 R
        # Calculate turning radius
        if self.d_theta:
            # print("odo:",self.d_l,self.d_r)
            if (self.d_l + self.d_r) == 0:  # 原地转向或静止
                self.Radius = 0
            else:
                if self.d_l * self.d_r > 0:  # 转向中心在walker之外
                    self.Radius = min(abs(self.d_l/self.d_theta), abs(self.d_r/self.d_theta)) + (self._wheel_base/2)
                else:  # 转向中心在walker之内
                    self.Radius = (self._wheel_base/2) - min(abs(self.d_l/self.d_theta), abs(self.d_r/self.d_theta))
        else:
            self.Radius = 0
            pass
        # print('Turning Radius: ', self.Radius, 'm;',self.d_theta)

        # 计算坐标变化dx, dy
        # First calculate the pose change under walker coordinate dx, dy
        if round(self.d_l, 3) == round(self.d_r, 3):  # Driving straight
            self.dx = 0.0
            self.dy = self.d_l
            self.V = self.dy / dt
        else:
            if round(self.d_l+self.d_r, 4) == 0:  # Stationary or turn-in-place
                self._dX, self._dY = 0.0, 0.0
                self.V = 0.0
            elif abs(self.d_l) > abs(self.d_r) and self.d_l > 0:  # 右前
                self.dx = self.Radius * (1 - math.cos(abs(self.d_theta)))
                self.dy = self.Radius * math.sin(abs(self.d_theta))
                self.V = abs(self.Radius * self.OMEGA)
            elif abs(self.d_l) > abs(self.d_r) and self.d_l <= 0:  # 右后
                self.dx = self.Radius * (1 - math.cos(abs(self.d_theta)))
                self.dy = -self.Radius * math.sin(abs(self.d_theta))
                self.V = -abs(self.Radius * self.OMEGA)
            elif abs(self.d_r) > abs(self.d_l) and self.d_r > 0:  # 左前
                self.dx = self.Radius * (math.cos(abs(self.d_theta)) - 1)
                self.dy = self.Radius * math.sin(abs(self.d_theta))
                self.V = abs(self.Radius * self.OMEGA)
            elif abs(self.d_r) > abs(self.d_l) and self.d_r <= 0:  # 左后
                self.dx = self.Radius * (math.cos(abs(self.d_theta)) - 1)
                self.dy = -self.Radius * math.sin(abs(self.d_theta))
                self.V = -abs(self.Radius * self.OMEGA)
        # print('dx=', self.dx, 'm;  dy=', self.dy, 'm;  dθ=', self.d_theta / math.pi * 180, '°')

        # Walker坐标系下的坐标变化dx,dy → 绝对坐标系下的坐标变化 dX, dY
        # Convert the dx, dy to the global pose change dX, dY
        # dX = cosθ·dx - sinθ·dy,  dY = sinθ·dx + cosθ·dy
        self._dX = self.dx * math.cos(self._THETA) - self.dy * math.sin(self._THETA)
        self._dY = self.dx * math.sin(self._THETA) + self.dy * math.cos(self._THETA)
        self._X += self._dX
        self._Y += self._dY
        self._THETA += self.d_theta
        if self._THETA > math.pi:
            self._THETA -= 2 * math.pi
        elif self._THETA <= -math.pi:
            self._THETA += 2 * math.pi
        # print("X,Y,theta",self.X,self.Y,self.THETA)
        self._previous_time = currnt_time
        return (self._X, self._Y, self._THETA)

    def getROS_speed(self):
        return (self.V, self.OMEGA, self.Radius)

    def getROS_XYTHETA(self):
        return (self._Y, -self._X, self._THETA)

    def get_dxdydtheta(self):
        return (self.dy, -self.dx, self.d_theta)

    def getTurningRadius(self):

        return self.Radius


if __name__ == "__main__":
    # odo = Odometry(X=0.0, Y=0.0, THETA=0.0, Odo_l=0, Odo_r=0)
    # newPos = odo.updatePose(4096, 0)
    # print('X:', newPos[0], 'm;   Y:', newPos[1], 'm;   THETA:', newPos[2] / math.pi * 180, '°;')
    pass
