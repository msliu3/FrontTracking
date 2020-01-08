#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   DemonstrationProcess.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/9 22:24   msliu      1.0         OOP for Demo and process, the output is theta and temperature
"""
import os, sys

BASE_DIR = os.path.dirname(os.path.abspath("/home/msliu/catkin_ws/src/neo_front_following/src/FootDetector"))
sys.path.append(BASE_DIR)
import serial
import numpy as np
from PIL import Image
import cv2 as cv
from FootDetector import ProcessFunc as pf, FootInformation as foot

from FootDetector.IRCamera import IRCamera


def singleton(cls, *args, **kw):
    instances = {}

    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]

    return _singleton


@singleton
class DemonProcess(object):
    head_size = 4
    resource = os.path.abspath(
        os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".." + os.path.sep + "resource") + "/output.avi"
    print(resource)

    def __init__(self, scope=30, fps=5, output_path=resource):
        self.env = -1
        self.scope = scope
        self.ir_array_data = np.array((32 * self.scope, 24 * self.scope))

        ir_camera = IRCamera()
        port_name, baud_rate = ir_camera.get_portname_baudrate()
        self.serial = serial.Serial(port_name, baud_rate, timeout=None)

        self.out = cv.VideoWriter(output_path, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps,
                                  (32 * self.scope, 24 * self.scope))
        self.foot = foot.FootInformation()
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

    def demonstrate_data(self, ir_data, filter_list, filter_num=5, zoom_filter=Image.BILINEAR):
        """
            接收一帧（1540*2个字节，32x24=768 （768+2）x2 = 1540）数据，
            数据的物理意义是温度，一个32*24的温度矩阵
            根据温度的取值范围映射成图像（0-255）
            再根据放大滤镜，放大一定倍数
            ------------------------------------------------------------
            其中在拿到数据之后，用了一个IIR滤波器，跟之前帧的数据取平均值
        :param filter_num:
        :param filter_list:
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

        # 传感器返回给我的环境温度没什么参考性，肯能指的是传感器的工作环境温度，并不能帮助我做判断，这里直接pop掉
        self.env = temp_data.pop()

        temp_data = self.__fix_pixel(temp_data, 6, 9)
        for i in temp_data:
            temperature.append(i)
        max_temp = max(temp_data)
        min_temp = min(temp_data)

        # 这是一个时间上的FIR，问题在于均值之后会出现结果值大于最大值的情况
        # filter_list.append(np.array(temp_data).reshape(24, 32))
        # if len(filter_list) > filter_num:
        #     filter_list.pop(0)
        # temp_data = pf.filter_for_ir(filter_list).tolist()[0]

        is_foot = pf.detect_is_foot(temperature)
        if not is_foot:
            return None, None, False

        result, flag, kmeans_env = pf.k_means_detect(temperature)
        # print("kmeans env:", kmeans_env)
        # print("foot is %d" % flag)
        for i in range(len(temp_data)):
            # demo1 two methods to demonstrate IR data
            # if result[i] == flag:
            #     temp_data[i] = int((temp_data[i] - min_temp) / (max_temp - min_temp) * 155) + 100
            # else:
            #     temp_data[i] = int((temp_data[i] - min_temp) / (max_temp - min_temp) * 100)

            temp_data[i] = int((temp_data[i] - min_temp) / (max_temp - min_temp) * 255)

        # list的顺序并不是图像顺序，需要reshape进行变形
        np_data = np.array(temp_data).reshape(24, 32)
        # np_data = np.array(temperature).reshape(24, 32)
        temperature = np.array(temperature, np.float32).reshape(24, 32)

        # zero = np.zeros((24,32))我希望是红蓝配色
        rgb_data = np.array([np_data, np_data, np_data], np.uint8).reshape(3, -1)
        rgb_data = rgb_data.T.reshape(24, 32, 3)

        # 分别是均值滤波器（当前使用）、中位数滤波器、高斯滤波器
        rgb_data = pf.image_processing_mean_filter(rgb_data, kernel_num=3)
        # rgb_data = cv.medianBlur(rgb_data,7)
        # rgb_data = cv.GaussianBlur(rgb_data, (3, 3), 0)

        image2 = Image.fromarray(rgb_data)

        im = image2.resize((32 * self.scope, 24 * self.scope), zoom_filter)
        array = np.array(im)

        return temperature, array, True

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

    def binary_image(self, np_ir, threshold=127):
        """

        threshold = 106可调
        findContours(thresh,mode,method), mode 和 method 可调

        这个函数用来以后调这个值

        :param threshold:
        :param np_ir:
        :return:
        """
        gary = cv.cvtColor(np_ir, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gary, threshold, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        # ret, thresh = cv.threshold(gary, threshold, 255, cv.THRESH_BINARY)
        # thresh2 = cv.adaptiveThreshold(gary, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 11, 2)
        contours, hierarchy = cv.findContours(thresh, 1, 2)
        cv.drawContours(np_ir, contours, -1, (255, 0, 0), 3)
        return np_ir, contours

    def find_foot_ankle(self, np_ir, contours):
        foot_pattern = pf.select_contours(np_ir, contours)
        if len(foot_pattern) == 2:
            self.foot.draw_and_obtain_element(np_ir, foot_pattern)
            self.foot.assign_data_to_draw()
        # if len(foot_pattern) == 2:
        #     # print("have foot_pattern")
        #     left_rect = 0.0
        #     left_line = 0.0
        #     right_rect = 0.0
        #     right_line = 0.0
        #     for item in foot_pattern:
        #         rect = pf.draw_min_rectangle(np_ir, item)
        #         """
        #             rect = cv.minAreaRect()
        #             最小外接矩形，返回值为一个数组
        #             rect[ 0 ] : 中心点 x,y
        #             rect[ 1 ] : size width and height
        #             rect[ 2 ] : angle
        #                     定义域是[-90, 0)
        #                     以矩形最底端画平行线，右侧水平夹角的角度负值
        #
        #         """
        #
        #         line_angle = pf.draw_min_line(np_ir, item)
        #
        #         x, y, w, h = pf.draw_normal_rectangle(np_ir, item)
        #
        #         if x < (32 * self.scope) / 2:
        #             left_rect = (90 + rect[2])
        #             left_line = (90 + line_angle)
        #             print("left: ", left_rect, left_line)
        #             cv.putText(np_ir, str(int(90 + rect[2])), (100, 100), cv.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255),
        #                        2,
        #                        cv.LINE_AA)
        #         else:
        #             right_rect = -rect[2]
        #             right_line = (90 - line_angle)
        #             print("right: ", right_rect, right_line)
        #             cv.putText(np_ir, str(int(- rect[2])), (32 * self.scope - 500, 100), cv.FONT_HERSHEY_SIMPLEX, 2,
        #                        (255, 255, 255), 1,
        #                        cv.LINE_AA)
        #     print("line:", left_line - right_line,"rect: ", left_rect - right_rect)
        #
        #     return item
        # else:
        #     return

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

                rect = pf.draw_min_rectangle(np_ir, foot_np)

                pf.draw_min_line(np_ir, foot_np)

                x, y = pf.draw_normal_rectangle(np_ir, foot_np)

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
        cv.namedWindow("The IR data", 0)
        cv.resizeWindow("The IR data", 32 * 30, 24 * 30)
        cv.imshow("The IR data", np_ir)

        self.out.write(np_ir)
        if cv.waitKey(1) == ord('q'):
            self.foot.draw_pic()
            self.out.release()
            cv.destroyAllWindows()
            self.serial.close()
            return -1
        else:
            if mode == 'frame-by-frame':
                cv.waitKey(-1)
            return 1

    def start_Demon(self, queue=None):
        head = []
        data = []
        filter_data = []
        rest_num = 5
        while True:
            s = self.serial.read(1).hex()
            if s != "":
                s = int(s, 16)
            head.append(s)

            if len(head) == self.head_size:
                if self.check_head_data(head):
                    temp = self.serial.read(1540)
                    data.append(temp.hex())
                    head.clear()
                else:
                    head.pop(0)

                # 将读到的数据进行展示
                if len(data) == rest_num:
                    temp, ir_np, foot_flag = self.demonstrate_data(data[rest_num - 1], filter_data,
                                                                   filter_num=2)  # ,zoom_filter=Image.HAMMING
                    # ir_np = pf.draw_hist(ir_np)
                    if foot_flag:
                        # filter is effective
                        ir_np = pf.image_processing_mean_filter(ir_np, kernel_num=16)

                        # pf.show_temperature(temp)
                        # ir_np = pf.image_processing_contrast_brightness(ir_np, 1.2, -0.8)
                        ir_np, contours = self.binary_image(np.array(ir_np))
                        if queue is not None:
                            queue.put(self.foot, block=False)
                            self.foot.clear_current_info()
                        self.find_foot_ankle(ir_np, contours)
                        if self.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
                            break

                    # ir_np, contours = dp.binary_image(np.array(ir_np))
                    # dp.find_foot_ankle(ir_np, contours)
                    # if dp.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
                    #     break
                    data.pop(rest_num - 1)
                    data.pop(0)


if __name__ == '__main__':
    pd = DemonProcess()
    pd.start_Demon()
