import datetime
import time
from threading import Thread
from DigitalDriver import DigitalServoDriver_linux as DsD
from DigitalDriver import DriverMonitor_zhuzhi as DM
from DigitalDriver import odometry_zhuzhi as odo
import matplotlib.pyplot as plt
import serial
import math

class ControlDriver(Thread):

    def __init__(self, V=0.0, OMEGA=0.0, record_mode=False, position_mode=False, left_right=1):
        """
        :param radius_wheel:
        :param record_mode:
        :param radius:
        :param left_right:
            如果发现 左右轮数据反了
            将 0 改为 1
            或 1 改为 0
        """
        Thread.__init__(self)
        self.radius_wheel = 85.00   #车轮半径
        self.wheel_base = 540.00    #轮距
        self.record_mode = record_mode
        self.position_mode = position_mode
        self.speed = V  #线速度
        self.omega = OMEGA  #角速度
        self.position = [0.0, 0.0, 0.0]
        self.count = 0
        driver = DsD.DigitalServoDriver(left_right=left_right)
        self.left_right = left_right
        baud_rate = driver.baud_rate
        self.ser_l = serial.Serial(driver.left, baud_rate, timeout=0.05)    #左轮串口
        self.ser_r = serial.Serial(driver.right, baud_rate, timeout=0.05)   #右轮串口
        self.monitor_l = DM.DriverMonitor()
        self.monitor_r = DM.DriverMonitor()
        self.plot_x = [0.0]
        self.plot_y = [0.0]

        # 初始化时读取一次驱动器监控信息，记录初始时encoder位置
        # 读取左轮监控信息
        self.ser_l.write(bytes([0x80, 0x00, 0x80]))
        read_byte_l = self.ser_l.read(5)
        if read_byte_l[4] == 0x80:
            read_byte_l += self.ser_l.read(31)
        else:
            read_byte_l += self.ser_l.read(27)
        # 读取右轮监控信息
        self.ser_r.write(bytes([0x80, 0x00, 0x80]))
        read_byte_r = self.ser_r.read(5)
        if read_byte_r[4] == 0x80:
            read_byte_r += self.ser_r.read(31)
        else:
            read_byte_r += self.ser_r.read(27)

        # 初始化Odometry
        self.motorStatus_l = self.monitor_l.processData(read_byte_l)
        self.motorStatus_r = self.monitor_r.processData(read_byte_r)
        print('-------------------------------------------------------------------------------------------------------')
        print('Initial LEFT monitor: ', self.motorStatus_l)
        print('Initial RIGHT monitor:', self.motorStatus_r)
        Odo_l_init = self.motorStatus_l['FeedbackPosition']
        Odo_r_init = self.motorStatus_r['FeedbackPosition']
        print('init: ', Odo_l_init, Odo_r_init)
        print('-------------------------------------------------------------------------------------------------------')
        self.odo = odo.Odometry(X=0.0, Y=0.0, THETA=0.0, Odo_l=Odo_l_init, Odo_r=Odo_r_init, plot=False)
        # time.sleep(2)

    def get_wheel_speed(self):
        # 计算两轮线速度
        # v_r = (2V + omega * wheelbase) / 2*wheel_radius
        # v_l = (2V - omega * wheelbase) / 2*wheel_radius
        vr = (2*self.speed + self.omega * self.wheel_base) / (2*self.radius_wheel)
        vl = (2*self.speed - self.omega * self.wheel_base) / (2*self.radius_wheel)
        return vl, vr

    def speed2rpm(self, speed):
        # 线速度 --> 角速度
        rpm = speed / (2 * math.pi * self.radius_wheel / 1000) * 60
        return int(rpm)

    def get_rpm_byte(self, rpm):
        rpm_byte = [0x06, ]
        rpm_hex = int(rpm / 6000 * 16384)

        if rpm_hex >= 0:
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        else:
            temp = 0xFFFF
            rpm_hex = temp + rpm_hex
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]

        rpm_byte.append(rpm[0])
        rpm_byte.append(rpm[1])
        parity = 0
        for item in rpm_byte:
            parity = parity + item
        if parity > 256:
            parity = parity & 0xFF
        rpm_byte.append(parity)
        return rpm_byte

    def control_part_speedmode(self):
        print("\n===================================== Start speed control ! =====================================")
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xc4, 0xc6]
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(start))
        self.ser_l.read(2)
        self.ser_r.write(bytes(start))
        self.ser_r.read(2)
        self.ser_l.write(bytes(pc_mode))
        self.ser_l.read(2)
        self.ser_r.write(bytes(pc_mode))
        self.ser_r.read(2)

        # 如果 record_mode 是 True，则停掉电机，只记录数据
        if self.record_mode:
            self.stopMotor()

        while True:
            # 读取驱动器监控信息
            vl, vr = self.get_wheel_speed()

            left = self.get_rpm_byte( self.speed2rpm(vl) )
            right = self.get_rpm_byte(-(self.speed2rpm(vr)))

            # print(left, right)
            self.ser_l.write(bytes(left))
            self.ser_l.flush()
            self.ser_l.read(2)
            self.ser_r.write(bytes(right))
            self.ser_r.flush()
            self.ser_r.read(2)
            time.sleep(0.05)
            try:
                watch = [0x80, 0x00, 0x80]
                # 左轮
                self.ser_l.write(bytes(watch))
                self.ser_l.flush()
                read_byte_l = self.ser_l.read(5)
                if read_byte_l[4] == 0x80:
                    read_byte_l += self.ser_l.read(31)
                else:
                    read_byte_l += self.ser_l.read(27)

                # 右轮
                self.ser_r.write(bytes(watch))
                self.ser_r.flush()
                read_byte_r = self.ser_r.read(5)
                if read_byte_r[4] == 0x80:
                    read_byte_r += self.ser_r.read(31)
                else:
                    read_byte_r += self.ser_r.read(27)

                if self.left_right == 1:
                    self.motorStatus_l = self.monitor_l.processData(read_byte_r)
                    self.motorStatus_r = self.monitor_r.processData(read_byte_l)
                else:
                    self.motorStatus_l = self.monitor_l.processData(read_byte_l)
                    self.motorStatus_r = self.monitor_r.processData(read_byte_r)

                self.odo.Odo_l = self.motorStatus_l['FeedbackPosition']
                self.odo.Odo_r = self.motorStatus_r['FeedbackPosition']

                # print('LEFT monitor: ', self.motorStatus_l)
                # print('RIGHT monitor:', self.motorStatus_r)

                # 更新位置
                self.position = self.odo.updatePose(-self.odo.Odo_l, self.odo.Odo_r)
                # print('Position:  X=', self.position[0], 'm;  Y=', self.position[1], 'm; THETA=', self.position[2] / math.pi * 180, '°;')

                if math.sqrt((self.position[0] - self.plot_x[-1]) ** 2 + (self.position[1] - self.plot_y[-1]) ** 2) > 0.1:
                    self.plot_x.append(self.position[0])
                    self.plot_y.append(self.position[1])

                # 若有故障
                if self.motorStatus_l["Malfunction"] or self.motorStatus_r["Malfunction"]:
                    # print('Left motor malfunction:  ' + self.motorStatus_l["Malfunction"])
                    # print('Right motor malfunction: ' + self.motorStatus_r["Malfunction"])
                    self.flag_end = 1

                # print("%f\t%f\t%f\t%f" % (time.time(), math.degrees(self.odo.THETA),self.position[0],self.position[1]))
                # print(
                #     "%f\t%f\t%f\t%f\t%f" % (
                #     time.time(), self.odo.get_dxdydtheta()[0], self.odo.get_dxdydtheta()[1], self.odo.getROS_XYTHETA()[0],
                #     self.odo.getROS_XYTHETA()[1]))
            except IndexError as i:
                print(i,"except")

            self.ser_l.reset_input_buffer()
            self.ser_r.reset_input_buffer()
        pass

    def control_part_positionmode(self):
        print("\n===================================== Start position control ! =====================================")
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xd0, 0xd2]
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(start))
        self.ser_l.read(2)
        self.ser_r.write(bytes(start))
        self.ser_r.read(2)
        self.ser_l.write(bytes(pc_mode))
        self.ser_l.read(2)
        self.ser_r.write(bytes(pc_mode))
        self.ser_r.read(2)
        pass

    def stopMotor(self):    #关闭电机，同时关闭刹车
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(end))
        self.ser_l.read(2)
        self.ser_r.write(bytes(end))
        self.ser_r.read(2)

        # 读取一帧驱动器监控信息
        watch = [0x80, 0x00, 0x80]
        # 左轮
        self.ser_l.write(bytes(watch))
        read_byte_l = self.ser_l.read(5)
        if read_byte_l[4] == 0x80:
            read_byte_l += self.ser_l.read(31)
        else:
            read_byte_l += self.ser_l.read(27)

        # 右轮
        self.ser_r.write(bytes(watch))
        read_byte_r = self.ser_r.read(5)
        if read_byte_r[4] == 0x80:
            read_byte_r += self.ser_r.read(31)
        else:
            read_byte_r += self.ser_r.read(27)

    def run(self):
        if self.position_mode:
            self.control_part_positionmode()
        else:
            self.control_part_speedmode()

    pass


if __name__ == '__main__':
    cd = ControlDriver(V=0, OMEGA=0.01, record_mode=False)
    cd.start()
    while True:
        new_V = input('New speed: ')
        new_OMEGA = input('New OMEGA: ')
        cd.speed = new_V
        cd.omega = new_OMEGA
        time.sleep(0.1)
