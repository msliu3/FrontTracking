"""
Created on Mon Aug  5 20:39:40 2019

#用python写一个程序读取红外摄像头的数据，并加入模糊滤镜

@author: Liu Mingshan
"""
import serial
import numpy as np
from PIL import Image
import cv2 as cv

from FootDetector.IRCamera import IRCamera

headSize = 4
scope = 20
ir_array_data = np.array((32 * scope, 24 * scope, 3))
out = cv.VideoWriter('./resource/output.avi', cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10.0, (32 * scope, 24 * scope))


def check_head_data(head_data=[]):
    """
    This function is to detect the head of frame from IR Camera
    :param head_data: The read data could be frame.
    :return:
    """
    global headSize
    if len(head_data) != headSize:
        print('The length of head is not equal to 4')
        return False
    #    This is head ir_data from one frame
    head = [0x5A, 0x5A, 0x02, 0x06]
    for i in range(headSize):
        if head_data[i] != head[i]:
            return False
    return True


def fix_pixel(list=[], x=6, y=9):
    """
    :param list: temperature ir_list, the ir data source
    :param x: the position x from 1
    :param y: the position y from 1
    :return: the average temperature to fix one pixel

    """
    x -= 1
    y -= 1
    temp = (list[x - 1 + (y - 1) * 32] +
            list[x + (y - 1) * 32] +
            list[x + 1 + (y - 1) * 32] +
            list[x - 1 + y * 32] +
            list[x + 1 + y * 32] +
            list[x - 1 + (y + 1) * 32] +
            list[x + (y + 1) * 32] +
            list[x + 1 + (y + 1) * 32]) / 8
    list.insert(x + y * 32, temp)
    list.pop(x + y * 32 + 1)

    return list


def contour_feature(image_array):
    gray = cv.cvtColor(image_array, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray, 106, 255, 0)
    contours, hierarchy = cv.findContours(thresh, 1, 2)
    print(len(contours))
    for i in contours:
        hull = cv.convexHull(i)
        if hull.shape[0] < 10:
            print('the num is less than 10.')
            break
        foot = []
        for a in hull:
            if a[0][1] > 100:
                foot.append(a)
        # print(foot)
        foot_np = np.array(foot)
        rect = cv.minAreaRect(foot_np)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        cv.drawContours(image_array, [box], 0, (0, 0, 255), 2)

        # cv.drawContours(image_array, foot, -1, (255, 0, 0), 3)
        x, y, w, h = cv.boundingRect(foot_np)
        cv.rectangle(image_array, (x, y), (x + w, y + h), (0, 0, 255), 2)

    return image_array


def demonstrate_data(ir_data=''):
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
    fix_pixel(temp_data, 6, 9)
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

    im = image2.resize((32 * scope, 24 * scope), Image.BILINEAR)
    array = np.array(im)
    global ir_array_data, out
    ir_array_data = array
    # array = contour_feature(array)

    out.write(array)
    cv.imshow("image", array)
    # cv.waitKey(-1)
    return temperature


head = []
data = []
string = ''
# 这块是配置串口，将来可能需要自动检测
# 已通过IRCamera实现
ir_data = IRCamera()
with serial.Serial(ir_data.port_name, ir_data.baud_rate, timeout=None) as ser:
    while True:
        s = ser.read(1).hex()
        string = string + s
        if s != "":
            s = int(s, 16)

        head.append(s)
        if len(head) == headSize:
            if check_head_data(head):
                temp = ser.read(1540)
                data.append(temp.hex())

            for i in range(4):
                head.pop(0)
        # 将读到的数据进行展示
            if len(data) == 5:
                demonstrate_data(data[4])
                if cv.waitKey(1) == ord('q'):
                    break
                data.pop(4)
out.release()
cv.destroyAllWindows()
ser.close()
