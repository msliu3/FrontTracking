import numpy as np
import matplotlib.pyplot as plt
from DigitalDriver import ControlDriver as CD
import math

class Odometry:
    def __init__(self, X=0.0, Y=0.0, THETA=0.0, Odo_l=0, Odo_r=0, imu_yaw=0.0, use_imu = False, plot=False):
        self.Odo_l, self.Odo_r = Odo_l, Odo_r
        self.d_theta = 0.0
        self.d_l, self.d_r = 0.0, 0.0
        self.p_l, self.p_r = 0, 0
        self.Radius = 0.0
        self.X, self.Y = X, Y
        self.dX, self.dY = 0.0, 0.0
        self.THETA = THETA
        self.use_imu = use_imu
        self.imu_yaw = imu_yaw
        self.imu_yaw_p = imu_yaw
        self.plot = plot
        self.dx, self.dy = 0.0, 0.0
        print('X=', self.X, 'm;  Y=', self.Y, 'm;  THETA=', self.THETA / math.pi * 180, '°')

    # 更新里程计读取到的信息
    def updatePose(self, Odo_l, Odo_r, imu_yaw):
        self.Odo_l, self.Odo_r = Odo_l, Odo_r
        self.imu_yaw = imu_yaw
        # 计算两轮相对于上一时刻的位移
        self.d_l = ((self.Odo_l - self.p_l) / 4096) * 2 * math.pi * 0.085
        self.d_r = ((self.Odo_r - self.p_r) / 4096) * 2 * math.pi * 0.085
        # 计算dθ，逆时针为正，顺时针为负
        if not self.use_imu:
            self.d_theta = (self.d_r - self.d_l) / 0.54  # 左转>0, 右转<0
        else:
            # use yaw readings from IMU
            self.d_theta = self.imu_yaw - self.imu_yaw_p
        # 保存此时刻编码器数据
        self.p_l = self.Odo_l
        self.p_r = self.Odo_r
        self.imu_yaw_p = self.imu_yaw

        # update THETA (-pi, pi]
        self.THETA += self.d_theta
        if self.THETA > math.pi:
            self.THETA -= 2 * math.pi
        elif self.THETA <= -math.pi:
            self.THETA += 2 * math.pi

        # get turning raidus
        if self.d_theta:
            # print("odo:",self.d_l,self.d_r)
            if (self.d_l + self.d_r) == 0:
                self.Radius = 0
            else:
                if self.d_l * self.d_r > 0:  # 转向中心在walker之外
                    self.Radius = min(abs(self.d_l / self.d_theta), abs(self.d_r / self.d_theta)) + 0.27
                else:  # 转向中心在walker之内
                    self.Radius = 0.27 - min(abs(self.d_l / self.d_theta), abs(self.d_r / self.d_theta))
            # print('Turning Radius: ', self.Radius, 'm;')
        else:
            # print('No Turning!')
            self.Radius = 0
            pass

        # 计算坐标变化dX, dY
        if self.d_l == self.d_r:  # 直行
            self.dx = 0.0
            self.dy = self.d_l
        else:
            if (self.d_l + self.d_r) == 0:  # 原地转向或静止
                self.dX, self.dY = 0.0, 0.0
            elif abs(self.d_l) > abs(self.d_r) and self.d_l > 0:  # 右前
                self.dx = self.Radius * (1 - math.cos(abs(self.d_theta)))
                self.dy = self.Radius * math.sin(abs(self.d_theta))
            elif abs(self.d_l) > abs(self.d_r) and self.d_l <= 0:  # 右后
                self.dx = self.Radius * (1 - math.cos(abs(self.d_theta)))
                self.dy = -self.Radius * math.sin(abs(self.d_theta))
            elif abs(self.d_r) > abs(self.d_l) and self.d_r > 0:  # 左前
                self.dx = self.Radius * (math.cos(abs(self.d_theta)) - 1)
                self.dy = self.Radius * math.sin(abs(self.d_theta))
            elif abs(self.d_r) > abs(self.d_l) and self.d_r <= 0:  # 左后
                self.dx = self.Radius * (math.cos(abs(self.d_theta)) - 1)
                self.dy = -self.Radius * math.sin(abs(self.d_theta))
        # print('dx=', self.dx, 'm;  dy=', self.dy, 'm;  dθ=', self.d_theta / math.pi * 180, '°')

        # Walker坐标系下的坐标变化dx,dy → 绝对坐标系下的坐标变化 dX, dY
        # dX = cosθ·dx - sinθ·dy,  dY = sinθ·dx + cosθ·dy （参考复平面的2D旋转公式）
        self.dX = self.dx * math.cos(self.THETA) - self.dy * math.sin(self.THETA)
        self.dY = self.dx * math.sin(self.THETA) + self.dy * math.cos(self.THETA)

        # 更新绝对坐标系下坐标变化
        self.X += self.dX
        self.Y += self.dY

        return (self.X, self.Y, self.THETA)

    def getROS_XYTHETA(self):
        return (self.Y, -self.X, self.THETA)

    def get_dxdydtheta(self):
        return (self.dy, -self.dx, self.d_theta) # 弧度制

    def getTurningRadius(self):
        return self.Radius


if __name__ == "__main__":
    # odo = Odometry(X=0.0, Y=0.0, THETA=0.0, Odo_l=0, Odo_r=0)
    # newPos = odo.updatePose(4096, 0)
    # print('X:', newPos[0], 'm;   Y:', newPos[1], 'm;   THETA:', newPos[2] / math.pi * 180, '°;')
    pass
