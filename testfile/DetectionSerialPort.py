#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DetectionSerialPort.py
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/8 19:51   msliu      1.0         Detect Serial Port
"""

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


def detect_serials(vid=0x10c4, pid=0xea60):
    ports = serial.tools.list_ports.comports()
    port_cnt = 0
    port_list = []
    for port in ports:
        print_serial(port)
        if vid == port.vid and port.pid == pid:
            port_list.append(port)
            port_cnt += 1
    return port_cnt, port_list


def detect_serial(vid=0x10c4, pid=0xea60):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if vid == port.vid and port.pid == pid:
            return True, port.device
    return False, ""


if __name__ == '__main__':
    r, dev = detect_serials()
    if r:
        print(dev)
