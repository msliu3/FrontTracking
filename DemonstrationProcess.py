#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DemonstrationProcess.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/9 22:24   msliu      1.0         OOP for Demo and process, the output is theta and temperature
"""

# import lib
import serial
import numpy as np
from PIL import Image, ImageFilter
import cv2 as cv

from IRCamera import IRCamera


class DemonProcess(object):
    head_size = 4

    def __init__(self, scope):
        self.scope = scope
        self.ir_array_data = np.array((32 * self.scope, 24 * self.scope))

        ir_camera = IRCamera()
        port_name, baud_rate = ir_camera.get_portname_baudrate()

        self.__serial = serial.Serial(port_name, baud_rate, timeout=None)

        return

    def check_head_data(self, head_data):
        """
        This function is to detect the head of frame from IR Camera
        :param head_data: The read data could be frame.
        :return:
        """

        # if len(head_data) != self.head_size:
        #     print('The length of head is not equal to 4')
        #     return False

        #    This is head ir_data from one frame
        if head_data is None:
            head_data = []
        head = [0x5A, 0x5A, 0x02, 0x06]
        for i in range(self.head_size):
            if head_data[i] != head[i]:
                return False
        return True

    def demonstrate_data(self, ir_data=''):
        """

        :param ir_data: is a string whose length is
        :return:
        """
        temperature = []
        temp_data = []
        for i in range(769):
            t = (int(ir_data[i * 4 + 2:i * 4 + 4], 16) * 256 + int(ir_data[i * 4:i * 4 + 2], 16)) / 100
            temp_data.append(t)
        env = temp_data.pop()
        self.__fix_pixel(temp_data, 6, 9)
        for i in temp_data:
            temperature.append(int(i))
        maxtemp = max(temp_data)
        mintemp = min(temp_data)

        for i in range(len(temp_data)):
            temp_data[i] = int((temp_data[i] - mintemp) / (maxtemp - mintemp) * 255)

        npdata = np.array(temp_data).reshape(24, 32)
        temperature = np.array(temperature, np.float32).reshape(24, 32)

        # zero = np.zeros((24,32))我希望是红蓝配色
        rgbdata = np.array([npdata, npdata, npdata], np.uint8).reshape(3, -1)
        rgbdata = rgbdata.T.reshape(24, 32, 3)
        image2 = Image.fromarray(rgbdata)

        # 马赛克版本
        # im = image2.resize((32 * scope, 24 * scope), Image.HAMMING)

        im = image2.resize((32 * self.scope, 24 * self.scope), Image.BILINEAR)
        array = np.array(im)
        global ir_array_data, out
        ir_array_data = array
        # array = contour_feature(array)

        out.write(array)
        cv.imshow("image", array)
        # cv.waitKey(-1)
        return temperature, array

    def __fix_pixel(self, ir_list=[], x=6, y=9):
        """
        :param ir_list: temperature list, the ir data source
        :param x: the position x from 1
        :param y: the position y from 1
        :return: the average temperature to fix one pixel

        """
        x -= 1
        y -= 1
        temp = (ir_list[x - 1 + (y - 1) * 32] +
                ir_list[x + (y - 1) * 32] +
                ir_list[x + 1 + (y - 1) * 32] +
                ir_list[x - 1 + y * 32] +
                ir_list[x + 1 + y * 32] +
                ir_list[x - 1 + (y + 1) * 32] +
                ir_list[x + (y + 1) * 32] +
                ir_list[x + 1 + (y + 1) * 32]) / 8
        ir_list.insert(x + y * 32, temp)
        ir_list.pop(x + y * 32 + 1)

        return ir_list

    def record_video(self, fps, output_path="./resource/output.avi"):
        out = cv.VideoWriter(output_path, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps,
                             (32 * self.scope, 24 * self.scope))
        head = []
        data = []
        while True:
            s = self.__serial.read(1).hex()
            if s != "":
                s = int(s, 16)

            head.append(s)
            if len(head) == self.head_size:
                if self.check_head_data(head):
                    temp = self.__serial.read(1540)
                    data.append(temp.hex())

                for i in range(4):
                    head.pop(0)
            # 将读到的数据进行展示，这个data是一个链表，前4个算是我舍弃的数据，一直在操作5项
            if len(data) == 5:
                demonstrate_data(data[4])
                if cv.waitKey(1) == ord('q'):
                    break
                data.pop(4)
        out.release()
        cv.destroyAllWindows()
        self.__serial.close()
        return


pass
