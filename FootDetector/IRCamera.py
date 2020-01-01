#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   IRCamera.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/8 19:46   msliu      1.0         None
"""

import serial.tools.list_ports


class IRCamera(object):
    port_name = ''
    baud_rate = 460800  # sometimes it could be 115200

    def __init__(self, baud_rate=460800):
        self.baud_rate = baud_rate
        self.port_name, self.port_list = self.detect_serials("USB2.0-Serial") #USB2.0-Serial USB-SERIAL CH340
        print(self.port_name, self.baud_rate)
        return

    def print_serial(self, port):
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

    def detect_serials(self, description, vid=0x10c4, pid=0xea60):
        ports = serial.tools.list_ports.comports()
        port_cnt = 0
        port_list = []
        for port in ports:
            # self.print_serial(port)

            if port.description.__contains__(description):
                port_list = port.description
                port_path = port.device
                return port_path, port_list
            else:
                print("Cannot find the device: IR Camera")


            # print("%x and %x" % (port.vid, port.pid))
            # if vid == port.vid and port.pid == pid:
            #     port_list.append(port)
            #     port_cnt += 1
        #     这里我还不知道vid和pid是什么东西
        return None, None

    def get_portname_baudrate(self):
        return self.port_name, self.baud_rate


if __name__ == '__main__':
    ir_data = IRCamera()
    portname, baudrate = ir_data.get_portname_baudrate()
    print(portname, baudrate)
