#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DigitalServoDriver.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/15 21:55   msliu      1.0         None
"""

import serial.tools.list_ports


class DigitalServoDriver(object):

    def __init__(self, baud_rate = 57600):
        self.baud_rate = baud_rate
        self.port_name, self.port_list = self.detect_serials("Prolific USB-to-Serial Comm Port")
        print(self.port_name, self.baud_rate)
        return

    def detect_serials(self, description, vid=0x10c4, pid=0xea60):
        ports = serial.tools.list_ports.comports()
        port_cnt = 0
        port_list = []
        for port in ports:
            # self.print_serial(port)

            # 这有问题，如果两个串口如何检测，如何区分左右

            if port.description.__contains__(description):
                port_list = port.description
                port_path = port.device
                return port_path, port_list
            else:
                print("Cannot find the device: IR Camera")

    def simple_control_test(self):
        """

        :return:
        """

        return

    pass

if __name__ == '__main__':
    dsd = DigitalServoDriver()
