#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DemonstrationProcess.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/9 22:24   msliu      1.0         OOP for Demo and process, the output is theta and temperature
"""

import serial
import numpy as np
from PIL import Image
import cv2 as cv

from IRCamera import IRCamera


class DemonProcess(object):
    head_size = 4

    def __init__(self, scope=30, fps=5, output_path="./resource/output.avi"):
        self.env = -1
        self.scope = scope
        self.ir_array_data = np.array((32 * self.scope, 24 * self.scope))

        ir_camera = IRCamera()
        port_name, baud_rate = ir_camera.get_portname_baudrate()
        self.serial = serial.Serial(port_name, baud_rate, timeout=None)

        self.out = cv.VideoWriter(output_path, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps,
                                  (32 * self.scope, 24 * self.scope))
        return

    def check_head_data(self, head_data):
        """
        This function is to detect the head of frame from IR Camera
        :param head_data: The read data could be frame.
        :return:
        """
        if head_data is None:
            print("check_head_data: the head data is None")
            head_data = []
        head = [0x5A, 0x5A, 0x02, 0x06]
        for i in range(self.head_size):
            if head_data[i] != head[i]:
                # print("The head is not caught")
                return False

        return True

    def demonstrate_data(self, ir_data, zoom_filter=Image.BILINEAR):
        """

        :param zoom_filter: Image.HAMMING or Image.BILINEAR
        :param ir_data: is a string whose length is
        :return:
        """
        if len(ir_data) != 1540 * 2:
            # 正常传过来一个字节 0xa5 是一个字节，一个元素表示4位， 然后用string表示一个字母就是一个字节
            print("the array of ir_data is not 1540", len(ir_data))

        temperature = []
        temp_data = []
        for i in range(769):
            t = (int(ir_data[i * 4 + 2:i * 4 + 4], 16) * 256 + int(ir_data[i * 4:i * 4 + 2], 16)) / 100
            temp_data.append(t)
        self.env = temp_data.pop()
        self.__fix_pixel(temp_data, 6, 9)
        for i in temp_data:
            temperature.append(int(i))
        maxtemp = max(temp_data)
        mintemp = min(temp_data)

        for i in range(len(temp_data)):
            # temp_data[i] = int((temp_data[i] - mintemp) / (maxtemp - mintemp) * 255)
            temp_data[i] = int((temp_data[i] - 5) / (45 - 10) * 255)

        npdata = np.array(temp_data).reshape(24, 32)
        temperature = np.array(temperature, np.float32).reshape(24, 32)

        # zero = np.zeros((24,32))我希望是红蓝配色
        rgbdata = np.array([npdata, npdata, npdata], np.uint8).reshape(3, -1)
        rgbdata = rgbdata.T.reshape(24, 32, 3)
        image2 = Image.fromarray(rgbdata)

        im = image2.resize((32 * self.scope, 24 * self.scope), zoom_filter)
        array = np.array(im)

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

    def analysis_foot(self):
        theta_left, theta_right = -1, -1
        return theta_left, theta_right

    def binary_image(self, np_ir):
        """

        threshold = 106可调
        findContours(thresh,mode,method), mode 和 method 可调

        这个函数用来以后调这个值

        :param np_ir:
        :return:
        """
        gary = cv.cvtColor(np_ir, cv.COLOR_BGR2GRAY)
        ret, thresh = cv.threshold(gary, 127, 255, 0)
        contours, hierarchy = cv.findContours(thresh, 1, 2)
        cv.drawContours(np_ir, contours, -1, (255, 0, 0), 3)
        return np_ir, contours

    def find_foot(self, np_ir, contours):
        big_pattern = []
        # 先通过大小45000像素点，判断一下可不可能是脚
        for item in contours:
            area = cv.contourArea(item)
            if area > 45000:
                big_pattern.append(item)
        # self.print_contours(big_pattern)
        # 如果是分成了两个部分，那么判断是脚
        if len(big_pattern) == 2:

            for item in big_pattern:
                hull = cv.convexHull(item)

                if hull.shape[0] < 10:
                    print('the num is less than 10.')
                    break
                foot = []
                for a in hull:
                    if a[0][1] > 100:
                        foot.append(a)

                foot_np = np.array(foot)
                rect = cv.minAreaRect(foot_np)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cv.drawContours(np_ir, [box], 0, (0, 0, 255), 2)

                rows, cols = np_ir.shape[:2]
                [vx, vy, x, y] = cv.fitLine(foot_np, cv.DIST_L2, 0, 0.01, 0.01)
                lefty = int((-x * vy / vx) + y)
                righty = int(((cols - x) * vy / vx) + y)
                cv.line(np_ir, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)

                x, y, w, h = cv.boundingRect(foot_np)
                cv.rectangle(np_ir, (x, y), (x + w, y + h), (0, 0, 255), 2)

                if x < (32 * self.scope) / 2:
                    cv.putText(np_ir, str(int(90 + rect[2])), (100, 100), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255),
                               2,
                               cv.LINE_AA)
                else:
                    cv.putText(np_ir, str(int(- rect[2])), (32 * self.scope - 500, 100), cv.FONT_HERSHEY_SIMPLEX, 2,
                               (255, 255, 255), 1,
                               cv.LINE_AA)

            return big_pattern
        else:
            return

    def calculate_theta_direct(self, foot_pattern, np_ir):

        return

    def print_contours(self, contours):
        print("------------------------------------")
        print("the num of contours:%d" % len(contours))
        for item in contours:
            area = cv.contourArea(item)
            print("the area of :", area)

        return

    def demo_record(self, np_ir, mode='continuous'):
        """

        :param np_ir:
        :param mode: continuous and frame-by-frame
        :return:
        """

        # 在这里考虑一下如何，可以既填数又填string

        cv.imshow("The IR data", np_ir)
        self.out.write(np_ir)
        if cv.waitKey(1) == ord('q'):
            self.out.release()
            cv.destroyAllWindows()
            self.serial.close()
            return -1
        else:
            if mode == 'frame-by-frame':
                cv.waitKey(-1)
            return 1


pass

if __name__ == '__main__':
    dp = DemonProcess()
    head = []
    data = []
    while True:
        s = dp.serial.read(1).hex()
        if s != "":
            s = int(s, 16)
        head.append(s)

        if len(head) == dp.head_size:
            if dp.check_head_data(head):
                temp = dp.serial.read(1540)
                data.append(temp.hex())
                head.clear()
            else:
                head.pop(0)

            # 将读到的数据进行展示
            if len(data) == 5:
                temp, ir_np = dp.demonstrate_data(data[4])
                ir_np, contours = dp.binary_image(ir_np)
                dp.find_foot(ir_np, contours)
                if dp.demo_record(ir_np) == -1:  # , 'frame-by-frame'
                    break
                data.pop(4)
