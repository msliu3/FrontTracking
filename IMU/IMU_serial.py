#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os, sys
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)
print(type(pwd))
print(father_path)

import math
import time
import struct
import serial
import binascii
import threading
import tkinter as tk
from datetime import datetime
import serial.tools.list_ports


def print_serial(port):
    print("---------------[ %s ]---------------" % port.name)
    print("Path: %s" % port.device)
    print("Descript: %s" % port.description)
    print("HWID: %s" % port.hwid)
    if not None == port.manufacturer:
        print("Manufacture: %s" % port.manufacturer)
    if not None == port.product:
        print("Product: %s" % port.product)
    if not None == port.interface:
        print("Interface: %s" % port.interface)
    print()


def detect_serials(description="target device", vid=0x10c4, pid=0xea60):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print_serial(port)
        if port.description.__contains__(description):
            port_path = port.device
            return port_path
        else:
            print("Cannot find the target device: %s" % description)
    return None


# 配置类
class Config:
    # 端口号
    serialPort = detect_serials(description="USB2.0-Serial")
    # 波特率
    baudRate = 115200
    # 在传感器数据中，最小的包长度是11个字节
    minPackageLen = 11


# 传感器数据读取类
class SensorReader:
    def __init__(self):
        self.port = serial.Serial(Config.serialPort, Config.baudRate)
        self.port.close()
        if not self.port.isOpen():
            self.port.open()
        self.receiveBuffer = bytearray()
        self.working = False

    # 打开
    def open(self):
        if not self.port.isOpen():
            self.port.open()

    # 关闭
    def close(self):
        self.port.close()

    # 发送数据
    def send(self, data):
        self.port.write(data)

    # 接收数据
    def receive(self):
        while self.working:
            # 休眠一个微小的时间，可以避免无谓的CPU占用，在windows上实验时直接从12%下降到接近于0%
            time.sleep(0.001)
            count = self.port.inWaiting()
            if count > 0:
                s = self.port.read(count)
                self.receiveBuffer += s

                # 开始工作

    def start(self):
        # 开始数据读取线程
        t = threading.Thread(target=self.receive)
        print("start reading...")
        # 将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    # 停止工作
    def stop(self):
        self.working = False


# 数据解析类
class DataParser:
    def __init__(self, sensorReader, myUI, displauUI=False):
        self.r = sensorReader
        self.u = myUI
        self.displayUI = displauUI
        self.working = False
        self.TimeStart = datetime.now()
        self.iniVariable()

    # 初始化解析丰关的变量
    def iniVariable(self):
        self.ChipTime = [0, 0, 0, 0, 0, 0, 0]
        self.a = [0, 0, 0, 0]
        self.w = [0, 0, 0, 0]
        self.Angle = [0, 0, 0, 0]
        self.h = [0, 0, 0, 0]
        self.Port = [0, 0, 0, 0]
        self.LastTime = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.Temperature = 0
        self.Pressure = 0
        self.Altitude = 0
        self.GroundVelocity = 0
        self.GPSYaw = 0
        self.GPSHeight = 0
        self.Longitude = 0
        self.Latitude = 0

    # 流逝的毫秒数，返回浮点数
    def elapseMilliSeconds(self):
        now = datetime.now()
        seconds = time.mktime(now.timetuple()) - time.mktime(self.TimeStart.timetuple())
        # 微秒数的差值
        microSeconds = now.microsecond - self.TimeStart.microsecond
        return seconds * 1000 + microSeconds / 1000.0

    # 流逝的秒数，返回浮点数
    def elapseSeconds(self):
        return self.elapseMilliSeconds() / 1000

    # 在缓冲数据中找到第一个包的起始位置
    def findFirstPackage(self, buffer):
        i = 0
        while True:
            if buffer[i] == 0x55 and (buffer[i + 1] & 0x50) == 0x50:
                return i
            if i + 2 >= len(buffer):
                return -1
            i += 1

    # 处理数据
    def handle(self):
        # 处理接收到的数据
        while self.working:
            text = ''
            # 显示当前收到的数据
            dataLen = len(self.r.receiveBuffer)
            if dataLen >= Config.minPackageLen:
                # 去掉第1个包头前的数据
                headerPos = self.findFirstPackage(self.r.receiveBuffer)
                text += '包头偏移：' + str(headerPos) + '\r\n'
                if headerPos >= 0:
                    if headerPos > 0:
                        self.r.receiveBuffer[0:headerPos] = b''
                    # 取 Config.minPackageLen 整数倍长度的数据
                    if dataLen - headerPos >= Config.minPackageLen:
                        packageCount = int((dataLen - headerPos) / Config.minPackageLen)
                        if packageCount > 0:
                            cutLen = packageCount * Config.minPackageLen
                            text += '当前收到包数：' + str(packageCount) + '\r\n'
                            temp = self.r.receiveBuffer[0:cutLen]
                            # 按16进制字符串的形式显示收到的内容
                            hexStr = str(binascii.b2a_hex(temp))
                            text += '16进制原始据：' + hexStr[2:len(hexStr) - 1]
                            self.r.receiveBuffer[0:cutLen] = b''
                            # 在窗口的文本框中显示数据
                            if self.displayUI:
                                self.u.showData(text)
                            # 解析数据,逐个数据包进行解析
                            for i in range(packageCount):
                                beginIdx = int(i * Config.minPackageLen)
                                endIdx = int(i * Config.minPackageLen + Config.minPackageLen)
                                byteTemp = temp[beginIdx:endIdx]
                                # 校验和通过了的数据包才进行解析
                                if self.sbSumCheck(byteTemp):
                                    self.decodeData(byteTemp)
            # print("data handled")
            time.sleep(0.005)

    # 解码包中的数据
    def decodeData(self, byteTemp):
        # 记录当前的相对时间
        TimeElapse = self.elapseSeconds();
        # 将8个字节的数据解析成4个短整型
        Data = list(struct.unpack("hhhh", byteTemp[2:10]))

        if byteTemp[1] == 0x50:
            self.ChipTime[0] = (2000 + byteTemp[2])
            self.ChipTime[1] = byteTemp[3]
            self.ChipTime[2] = byteTemp[4]
            self.ChipTime[3] = byteTemp[5]
            self.ChipTime[4] = byteTemp[6]
            self.ChipTime[5] = byteTemp[7]
            self.ChipTime[6] = struct.unpack("h", byteTemp[8:10])[0]

        if byteTemp[1] == 0x51:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 16
            Data[1] = Data[1] / 32768.0 * 16
            Data[2] = Data[2] / 32768.0 * 16

            self.a[0] = Data[0]
            self.a[1] = Data[1]
            self.a[2] = Data[2]
            self.a[3] = Data[3]
            if ((TimeElapse - self.LastTime[1]) < 0.1):
                return
            self.LastTime[1] = TimeElapse

        if byteTemp[1] == 0x52:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 2000
            Data[1] = Data[1] / 32768.0 * 2000
            Data[2] = Data[2] / 32768.0 * 2000
            self.w[0] = Data[0]
            self.w[1] = Data[1]
            self.w[2] = Data[2]
            self.w[3] = Data[3]
            if ((TimeElapse - self.LastTime[2]) < 0.1):
                return
            self.LastTime[2] = TimeElapse

        if byteTemp[1] == 0x53:
            self.Temperature = Data[3] / 100.0
            Data[0] = Data[0] / 32768.0 * 180
            Data[1] = Data[1] / 32768.0 * 180
            Data[2] = Data[2] / 32768.0 * 180
            self.Angle[0] = Data[0]
            self.Angle[1] = Data[1]
            self.Angle[2] = Data[2]
            self.Angle[3] = Data[3]
            if ((TimeElapse - self.LastTime[3]) < 0.1):
                return
            self.LastTime[3] = TimeElapse

        if byteTemp[1] == 0x54:
            self.Temperature = Data[3] / 100.0
            self.h[0] = Data[0]
            self.h[1] = Data[1]
            self.h[2] = Data[2]
            self.h[3] = Data[3]
            if ((TimeElapse - self.LastTime[4]) < 0.1):
                return
            self.LastTime[4] = TimeElapse

        if byteTemp[1] == 0x55:
            self.Port[0] = Data[0]
            self.Port[1] = Data[1]
            self.Port[2] = Data[2]
            self.Port[3] = Data[3]

        if byteTemp[1] == 0x56:
            self.Pressure = struct.unpack("i", byteTemp[2:6])[0]
            self.Altitude = struct.unpack("i", byteTemp[6:10])[0] / 100.0

        if byteTemp[1] == 0x57:
            self.Longitude = struct.unpack("i", byteTemp[2:6])[0]
            self.Latitude = struct.unpack("i", byteTemp[6:10])[0]

        if byteTemp[1] == 0x58:
            self.GPSHeight = struct.unpack("h", byteTemp[2:4])[0] / 10.0
            self.GPSYaw = struct.unpack("h", byteTemp[4:6])[0] / 10.0
            self.GroundVelocity = struct.unpack("h", byteTemp[6:8])[0] / 1e3

        # 内嵌的输出函数，可以直接引用方法内部的各种变量，比如 TimeElapse 等
        def output():
            text = '系统时间：' + datetime.now().strftime("%Y-%m-%d %H:%M:%S") + "\r\n"
            text += '片上时间：' + str(self.ChipTime[0]) + "-" + str(self.ChipTime[1]) + "-" + str(
                self.ChipTime[2]) + "\r\n                    "
            text += str(self.ChipTime[3]) + ":" + str(self.ChipTime[4]) + ":" + str(self.ChipTime[5]) + "." + str(
                self.ChipTime[6]) + "\r\n"
            text += '相对时间：' + "%.3f" % TimeElapse + "\r\n\r\n"

            text += 'x轴加速度：' + "%.2f g" % self.a[0] + "\r\n"
            text += 'y轴加速度：' + "%.2f g" % self.a[1] + "\r\n"
            text += 'z轴加速度：' + "%.2f g" % self.a[2] + "\r\n\r\n"

            text += 'x轴角速度：' + "%.2f °/s" % self.w[0] + "\r\n"
            text += 'y轴角速度：' + "%.2f °/s" % self.w[1] + "\r\n"
            text += 'z轴角速度：' + "%.2f °/s" % self.w[2] + "\r\n\r\n"

            text += 'x轴角度：  ' + "%.2f °" % self.Angle[0] + "\r\n"
            text += 'y轴角度：  ' + "%.2f °" % self.Angle[1] + "\r\n"
            text += 'z轴角度：  ' + "%.2f °" % self.Angle[2] + "\r\n\r\n"

            text += 'x轴磁场： ' + "%.0f mG" % self.h[0] + "\r\n"
            text += 'y轴磁场： ' + "%.0f mG" % self.h[1] + "\r\n"
            text += 'z轴磁场： ' + "%.0f mG" % self.h[2] + "\r\n\r\n"

            text += '温   度：' + "%.2f ℃" % self.Temperature + "\r\n"
            text += '气   压：' + "%.0f Pa" % self.Pressure + "\r\n"
            text += '高   度：' + "%.2f m" % self.Altitude + "\r\n\r\n"

            text += '经   度：' + "%.0f°" % (self.Longitude / 10000000) + "%.5f\'" % (
                        (self.Longitude % 10000000) / 1e5) + "\r\n"
            text += '纬   度：' + "%.0f°" % (self.Latitude / 10000000) + "%.5f\'" % (
                        (self.Latitude % 10000000) / 1e5) + "\r\n"

            text += 'GPS高度：' + "%.1f m" % self.GPSHeight + "\r\n"
            text += 'GPS航向：' + "%.1f °" % self.GPSYaw + "\r\n"
            text += 'GPS地速：' + "%.3f km/h" % self.GroundVelocity + "\r\n\r\n"

            self.u.showText(text)

        # 输出解析得到的内容
        if self.displayUI:
            output()

    # 检查校验和
    def sbSumCheck(self, byteTemp):
        if (((byteTemp[0] + byteTemp[1] + byteTemp[2] + byteTemp[3] + byteTemp[4] + byteTemp[5] + byteTemp[6] +
              byteTemp[7] + byteTemp[8] + byteTemp[9]) & 0xff) == byteTemp[10]):
            # print('sum check ok!')
            return True
        else:
            print('sum check false!')
            return False

    # 开始工作
    def start(self):
        # 开启数据解析线程
        t = threading.Thread(target=self.handle)
        # 将当前线程设为子线程t的守护线程，这样一来，当前线程结束时会强制子线程结束
        t.setDaemon(True)
        self.working = True
        t.start()

    # 停止工作
    def stop(self):
        self.working = False


# 图形界面类
class MyUI:
    def __init__(self):
        # 创建显示传感器数据的窗口
        self.window = tk.Tk()
        self.window.title('WT901(JY901)传感器')
        self.window.geometry('760x840')

        self.frameTop = tk.Frame(self.window)
        self.frameTop.config(height=100, width=780)
        self.frameTop.place(x=10, y=0)

        self.frameBottom = tk.Frame(self.window)
        self.frameBottom.config(height=720, width=780)
        self.frameBottom.place(x=10, y=110)

        # 创建显示数据的文本框
        self.dataBox = tk.Text(self.frameTop, bg='white', font=('Arial', 12))
        self.dataBox.place(x=4, y=4)

        self.textBox = tk.Text(self.frameBottom, height=700, bg='white', font=('Arial', 12))
        self.textBox.place(x=4, y=4)

    # 开启UI
    def start(self):
        # 开启窗口主循环
        self.window.mainloop()

    # 显示数据
    def showData(self, data):
        self.dataBox.delete(0.0, tk.END)
        self.dataBox.insert(tk.INSERT, data)

    # 显示文本
    def showText(self, text):
        self.textBox.delete(0.0, tk.END)
        self.textBox.insert(tk.INSERT, text)


def euler_to_quaternion(roll, pitch, yaw):
    w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    return [w, x, y, z]


def quaternion_to_euler(w, x, y, z):
    roll = math.atan2(2*(w*x+y*z), 1-2*(x**2+y**2))
    yaw = math.atan2(2*(w*z + x*y), 1-2*(z**2+y**2))
    pitch = math.asin(2*(w*y-x*z))
    return [roll, pitch, yaw]


def imu_callback(imu, pub):
    pub.publish(imu)


# 主线程
if __name__ == '__main__':

    run_ROS = True
    displayUI = False

    # 创建串口操作对象
    r = SensorReader()
    r.start()
    # 创建UI对象
    u = MyUI()
    # 创建数据解析对象
    p = DataParser(r, u, displauUI=displayUI)
    p.start()
    if displayUI:
        u.start()   # 启动UI

    try:
        if run_ROS:
            import rospy
            from sensor_msgs.msg import Imu, MagneticField
            from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

            # print("success")
            rospy.init_node('imu_node')
            r = rospy.Rate(20)
            imu_raw = Imu()
            mag = MagneticField()
            imu_raw_pub = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
            mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size=10)
            # Cartographer subscribes to topic "imu", whereas the madgwick_filter publish "imu/data"
            # Here we echo the "imu/data" topic and publish it as topic "imu"
            # imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
            # imu_sub = rospy.Subscriber("imu/data", Imu, imu_callback, imu_pub)

            while not rospy.is_shutdown():
                # Publish imu message on ROS
                imu_raw.header.stamp = rospy.Time.now()
                imu_raw.header.frame_id = "imu_link"
                quat = euler_to_quaternion(0, 0, p.Angle[2] / 180 * math.pi)
                # print("Yaw: ", p.Angle[2])
                # quat = euler_to_quaternion(p.Angle[0]/180*math.pi,
                #                            p.Angle[1]/180*math.pi,
                #                            p.Angle[2]/180*math.pi)
                imu_raw.orientation = Quaternion(quat[1], quat[2], quat[3], quat[0])
                imu_raw.angular_velocity = Vector3(p.w[0] / 180 * math.pi,
                                                   p.w[1] / 180 * math.pi,
                                                   p.w[2] / 180 * math.pi)
                imu_raw.angular_velocity_covariance[0] = -1
                imu_raw.linear_acceleration = Vector3(p.a[0] / 9.81,
                                                      p.a[1] / 9.81,
                                                      p.a[2] / 9.81)
                imu_raw.linear_acceleration_covariance[0] = -1
                imu_raw_pub.publish(imu_raw)

                # Publish magnetic field message on ROS
                mag.header.stamp = rospy.Time.now()
                mag.header.frame_id = "mag_link"
                mag.magnetic_field.x = p.h[0]
                mag.magnetic_field.y = p.h[1]
                mag.magnetic_field.z = p.h[2]
                mag_pub.publish(mag)

                r.sleep()

    except rospy.ROSInterruptException:
        pass