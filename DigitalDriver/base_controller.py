#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

import time
from threading import Thread
from DigitalDriver import DigitalServoDriver_linux as DsD
from DigitalDriver import DriverMonitor_zhuzhi as DM
from DigitalDriver import WheelEncoderOdometry as odo
from DigitalDriver import OdometryIMU_zhuz as odo_imu
import serial
import math

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class ControlDriver(Thread):

    def __init__(self, V=0.0, OMEGA=0.0, record_mode=False, left_right=1):
        # :param radius_wheel:
        # :param record_mode:
        # :param radius:
        # :param left_right:
        #     如果发现 左右轮数据反了
        #     将 0 改为 1 或 1 改为 0
        Thread.__init__(self)
        self.radius_wheel = 85.00   #车轮半径/mm
        self.wheel_base = 540.00    #轮距/mm
        self.record_mode = record_mode
        self.speed = V  #线速度 m/s
        self.omega = OMEGA  #角速度 rad/s
        self.position = [0.0, 0.0, 0.0]
        self.count = 0
        self.is_stopped = record_mode
        driver = DsD.DigitalServoDriver()
        self.left_right = left_right
        baud_rate = driver.baud_rate
        if left_right == 1:
            self.ser_l = serial.Serial(driver.left, baud_rate, timeout=0.05)    #左轮串口
            self.ser_r = serial.Serial(driver.right, baud_rate, timeout=0.05)   #右轮串口
        else:
            self.ser_l = serial.Serial(driver.right, baud_rate, timeout=0.05)  # 左轮串口
            self.ser_r = serial.Serial(driver.left, baud_rate, timeout=0.05)  # 右轮串口
        self.monitor_l = DM.DriverMonitor()
        self.monitor_r = DM.DriverMonitor()
        self.plot_x = [0.0]
        self.plot_y = [0.0]

        # 初始化时读取一次驱动器监控信息，记录初始时encoder位置
        read_byte_l = self.read_monitor(self.ser_l)
        read_byte_r = self.read_monitor(self.ser_r)

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

    def read_monitor(self, ser):
        ser.write(bytes([0x80, 0x00, 0x80]))
        read_byte = ser.read(5)
        if read_byte[4] == 0x80:
            read_byte += ser.read(31)
        else:
            read_byte += ser.read(27)
        return read_byte

    def change_speed(self, v, omega):
        if self.is_stopped and v+omega!=0:
            self.start_motor()

        # The following lines are used if you wish to disable brake when
        # the walker is stationary. However this could be problematic when
        # using xbox controller.
        # if(self.speed + self.omega != 0) and (v + omega == 0):
        #     self.stop_motor()
        # elif( (v + omega) != 0 and (self.speed + self.omega == 0)):
        #     self.start_motor()

        self.speed = v
        self.omega = omega
        time.sleep(0.05)

    def get_wheel_speed(self):
        # 计算两轮线速度
        # v_r = (2V + omega * wheelbase) / 2
        # v_l = (2V - omega * wheelbase) / 2
        vl = (2*self.speed - self.omega * self.wheel_base / 1000) / 2
        vr = (2*self.speed + self.omega * self.wheel_base / 1000) / 2
        return vl, vr

    def speed2rpm(self, speed):
        # 线速度 --> 角速度
        rpm = speed / (2 * math.pi * self.radius_wheel / 1000) * 60
        return int(rpm)

    def rpm2byte(self, rpm):
        rpm_byte = [0x06, 0x00, 0x88, 0x8e]
        rpm_hex = int(rpm / 6000 * 16384)
        if rpm_hex >= 0:
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        else:
            temp = 0xFFFF
            rpm_hex = temp + rpm_hex
            rpm = [(rpm_hex & 0xFF00) >> 8, (rpm_hex & 0x00FF)]
        rpm_byte[1] = rpm[0]
        rpm_byte[2] = rpm[1]
        rpm_byte.pop(3)
        last = 0
        for item in rpm_byte:
            last = last + item
        if last > 256:
            last = last & 0xFF
        rpm_byte.append(last)
        return rpm_byte

    def control_part_speedmode(self):
        print("\n===================================== Start speed control ! =====================================")
        self.start_motor()
        # 如果 record_mode 是 True，则停掉电机，只记录数据
        if self.record_mode:
            self.stop_motor()

        while True:
            vl, vr = self.get_wheel_speed()
            # print("left: ", vl, "; right: ", vr)
            vl = self.speed2rpm(vl)
            vr = self.speed2rpm(vr)
            left = self.rpm2byte(-vl)
            right = self.rpm2byte(vr)
            # print("byte_left: ", left, "byte_right: ", right)
            self.ser_l.write(bytes(left))
            self.ser_l.flush()
            self.ser_l.read(2)
            self.ser_r.write(bytes(right))
            self.ser_r.flush()
            self.ser_r.read(2)
            time.sleep(0.05)
            try:
                read_byte_l = self.read_monitor(self.ser_l)
                read_byte_r = self.read_monitor(self.ser_r)
                self.motorStatus_l = self.monitor_l.processData(read_byte_l)
                self.motorStatus_r = self.monitor_r.processData(read_byte_r)
                Odo_l = self.motorStatus_l['FeedbackPosition']
                Odo_r = self.motorStatus_r['FeedbackPosition']

                # 更新位置
                self.position = self.odo.updatePose(-Odo_l, Odo_r)
                # print('Position:  X=', self.position[0], 'm;  Y=', self.position[1], 'm;
                #       THETA=', self.position[2] / math.pi * 180, '°;')

                if math.sqrt((self.position[0]-self.plot_x[-1])**2+(self.position[1]-self.plot_y[-1])**2)>0.1:
                    self.plot_x.append(self.position[0])
                    self.plot_y.append(self.position[1])

                # 若有故障
                if self.motorStatus_l["Malfunction"] or self.motorStatus_r["Malfunction"]:
                    # print('Left motor malfunction:  ' + self.motorStatus_l["Malfunction"])
                    # print('Right motor malfunction: ' + self.motorStatus_r["Malfunction"])
                    self.flag_end = 1

            except IndexError as i:
                print(i, "except")

            self.ser_l.reset_input_buffer()
            self.ser_r.reset_input_buffer()
        pass

    def start_motor(self):
        start = [0x00, 0x00, 0x01, 0x01]
        pc_mode = [0x02, 0x00, 0xc4, 0xc6]
        self.ser_l.write(bytes(start))
        self.ser_l.read(2)
        self.ser_r.write(bytes(start))
        self.ser_r.read(2)
        self.ser_l.write(bytes(pc_mode))
        self.ser_l.read(2)
        self.ser_r.write(bytes(pc_mode))
        self.ser_r.read(2)
        self.is_stopped = False

    def stop_motor(self):    #关闭电机，同时关闭刹车
        self.speed = 0
        self.omega = 0
        time.sleep(0.05)
        end = [0x00, 0x00, 0x00, 0x00]
        self.ser_l.write(bytes(end))
        self.ser_l.read(2)
        self.ser_r.write(bytes(end))
        self.ser_r.read(2)
        # 读取一帧驱动器监控信息
        read_byte_l = self.read_monitor(self.ser_l)
        read_byte_r = self.read_monitor(self.ser_r)

        self.is_stopped = True

    def ext_brake(self):
        self.speed = 0
        self.omega = 0
        pass

    def run(self):
        self.control_part_speedmode()

    def __del__(self):
        self.stop_motor()
        pass

    pass


def callback_vel(vel, cd):
    # callback function to change the speed when receives /cmd_vel topic
    speed = vel.linear.x
    omega = vel.angular.z
    if (speed != cd.speed) or (omega != cd.omega):
        # if velocity command is not zero and motors are stopped, start it
        print('Speed: %.3f, Omega: %.3f' % (speed, omega))
        cd.change_speed(speed, omega)
    else:
        pass


def imu_callback(imu, cd):
    def quaternion_to_euler(w, x, y, z):
        roll = math.atan(2 * (w * x + y * z) / (1 - 2 * (x ** 2 + y ** 2)))
        pitch = math.atan(2 * (w * z + x * y) / (1 - 2 * (z ** 2 + y ** 2)))
        yaw = math.atan(2 * (w * y - x * z))
        return [roll, pitch, yaw]

    cd.imu_yaw = quaternion_to_euler(imu.orientation.w,
                                     imu.orientation.x,
                                     imu.orientation.y,
                                     imu.orientation.z)[2]


if __name__ == '__main__':

    cd = ControlDriver(V=0.0, OMEGA=0.0, left_right=0, record_mode=True)
    cd.start()

    # while True:
    #     time.sleep(0.5)
    #     (x, y, theta) = cd.odo.getROS_XYTHETA()
    #     print('Position:  X= %.3f, Y= %.3f, THETA= %.3f°' % (x, y, theta / math.pi * 180))

    rospy.init_node('base_controller_node')
    r = rospy.Rate(10)
    vel_sub = rospy.Subscriber("cmd_vel", Twist, callback_vel, cd)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    odom = Odometry()
    pos_p = cd.odo.getROS_XYTHETA()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        dt = current_time - last_time
        dt = dt.to_sec()    # dt is a Duration() class, convert to float

        # Publish raw odometry data by the topic "/odom"
        pos = cd.odo.getROS_XYTHETA()
        x, y, theta = pos[0], pos[1], pos[2]
        dx, dy, dtheta = x-pos_p[0], y-pos_p[1], theta-pos_p[2]
        vx, vy, vtheta = dx/dt, dy/dt, dtheta/dt

        # Theta euler -> quaternion
        odom_quat = Quaternion()
        odom_quat.w = math.cos(0.5*theta)
        odom_quat.z = math.sin(0.5*theta)
        # publish Odometry over ROS
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x, y, 0.), odom_quat)
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vtheta))
        odom_pub.publish(odom)
        pos_p = pos

        last_time = current_time

        r.sleep()
