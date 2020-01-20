#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   gatherIMU.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/1/20 0:06   msliu      1.0      

@Description
------------
None
"""

# import lib
import serial
import serial.tools.list_ports
import re


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


def detect_serials(description, vid=0x10c4, pid=0xea60):
    ports = serial.tools.list_ports.comports()
    port_cnt = 0
    port_list = []
    for port in ports:
        print_serial(port)

        if port.description.__contains__(description):
            port_path = port.device
            return port_path
        else:
            print("Cannot find the device: IR Camera")

        # print("%x and %x" % (port.vid, port.pid))
        # if vid == port.vid and port.pid == pid:
        #     port_list.append(port)
        #     port_cnt += 1
    #     这里我还不知道vid和pid是什么东西
    return None


class ArduinoRead(object):
    def __init__(self):
        port_name = detect_serials(description="ttyACM") #CP2102 USB to UART Bridge Controller Arduino Mega 2560 (COM15)
        baud_rate = 115200
        print(port_name, baud_rate)
        self.serial = serial.Serial(port_name, baud_rate, timeout=None)
        self.imu_human = 0.0
        self.imu_robot = 0.0
        self.terminal_flage = False
        pass

    def terminal_thread(self):
        self.terminal_flage = True
        pass

    def reading_data_from_arduino(self):
        while not self.terminal_flage:
            data = self.serial.readline()
            line = str(data)
            list_data = re.findall("\d+\.\d+", line)
            print(list_data)
            if len(list_data) == 2:
                self.imu_human = list_data[0]
                self.imu_robot = list_data[1]


if __name__ == '__main__':
    ar = ArduinoRead()
    ar.reading_data_from_arduino()
