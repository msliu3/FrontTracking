#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   ProcessFunc.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/29 20:08   msliu      1.0      

@Description
------------
这个文件提供各种函数
"""
import numpy as np
import cv2 as cv
import sys
import math
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans


def get_contours(filename, threshhold=97):
    if type(filename) is type(''):
        img = cv.imread(filename, 1)
    else:
        img = filename
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(img_gray, threshhold, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    contours, hierarchy = cv.findContours(thresh, 2, 1)
    return img, contours


def select_contours(img, contours):
    cnt_list = sort_contours(img, contours)
    if len(cnt_list) <= 1:
        return cnt_list
    elif len(cnt_list) == 2:
        print("two")
        temp = []
        area1 = cv.contourArea(cnt_list[0])
        area2 = cv.contourArea(cnt_list[1])
        result = abs(area1 - area2) / area1
        if 0.8 < result < 1:
            cnt_list.append(2)
            cnt_list.append(2)
            cnt_list.append(2)
            return cnt_list
        for item in cnt_list:
            print(cv.contourArea(item))
            foot = get_foot_ankle(item)
            temp.append(foot)
        cnt_list = temp
        print("pass",result)
        return cnt_list
    elif len(cnt_list) >= 3:
        print("three", len(cnt_list))
        temp = []
        area1 = cv.contourArea(cnt_list[0])
        area2 = cv.contourArea(cnt_list[1])
        result = abs(area1 - area2) / area1
        print(result)
        if 0.5 < result < 1:
            cnt_list.append(1)
            return cnt_list
        else:
            for i in range(2):
                foot = get_foot_ankle(cnt_list[i])
                temp.append(foot)

            cnt_list = temp
            return cnt_list
    # if len(cnt_list) == 1:
    #     print("only one")
    #     point = get_convexity_point(cnt_list[0])
    #     l, r = segmentation_two_feet(img, point)
    #     cnt_list.pop(0)
    #     cnt_list.append(l)
    #     cnt_list.append(r)
    #     return cnt_list
    pass


def sort_contours(img, contours):
    cnt_list = []
    image_area = img.shape[0] * img.shape[1]
    # print("total:", image_area)
    contours = sorted(contours, key=lambda x: len(x), reverse=True)
    for item in contours:
        area = cv.contourArea(item)
        rate = area / image_area
        # print("area:", area, "rate", rate)
        cnt_list.append(item)
    return cnt_list


def segmentation_two_feet(img, convex_point):
    segment_point_x = convex_point[0][0][0]
    segment_point_y = convex_point[0][0][1]
    left_foot = np.array(img[segment_point_y:img.shape[0], 0:segment_point_x])
    image, contours = get_contours(left_foot, threshhold=127)
    sorted_point = sorted(contours, key=lambda x: len(x), reverse=True)
    left = get_foot_ankle(sorted_point[0])

    right_foot = np.array(img[segment_point_y:img.shape[0], segment_point_x:img.shape[1]])
    image, contours = get_contours(right_foot, threshhold=127)

    sorted_point = sorted(contours, key=lambda x: len(x), reverse=True)
    right = get_foot_ankle(sorted_point[0])
    for i in range(len(right)):
        right[i][0][0] += segment_point_x
    # cv.imwrite("../resource/temp.jpg", left_foot)
    # cv.drawContours(image, contours, -1, (255, 0, 0), 3)
    # cv.imwrite("../resource/temp-contours.jpg", left_foot)
    # cv.imshow("temp", image)

    return left, np.array(right)


def bigger_y(point):
    biggest_y = sys.maxsize
    for item in point:
        if item[0][1] < biggest_y:
            biggest_y = item[0][1]
    return biggest_y


def get_convexity_point(cnt, img=None):
    """

    :param img:
    :param cnt:
    :return:  ((x,y),distance)
    """
    hull = cv.convexHull(cnt, returnPoints=False)
    defects = cv.convexityDefects(cnt, hull)
    point = {}
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        point[tuple(cnt[f][0])] = d
        far = tuple(cnt[f][0])
        cv.circle(img, far, 5, [0, 0, 255], -1)
    sorted_point = sorted(point.items(), key=lambda x: x[1], reverse=True)
    # print(sorted_point)
    return sorted_point


def draw_three(image, cnt_list):
    return


def draw_normal_rectangle(img, np_point, color=(0, 0, 255)):
    x, y, w, h = cv.boundingRect(np_point)
    cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
    return x, y, w, h


def draw_min_rectangle(img, np_point, color=(0, 0, 255)):
    rect = cv.minAreaRect(np_point)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, color, 2)
    return rect


def draw_min_line(img, np_point, color=(0, 0, 255)):
    """

    :param img:
    :param np_point:
    :param color:
    :return: angle
        向右倾斜，贯穿一二三相线 为 负值
        向左倾斜，贯穿一二四相线 为 正值
        成角都是和水平线的成角
    """
    rows, cols = img.shape[:2]
    [vx, vy, x, y] = cv.fitLine(np_point, cv.DIST_L2, 0, 0.01, 0.01)
    lefty = int((-x * vy / vx) + y)
    righty = int(((cols - x) * vy / vx) + y)
    cv.line(img, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
    h = math.atan((lefty - righty) / (0 - cols + 1))
    angle = math.degrees(h)
    # print("rows: %d cols: %d" % (rows, cols))
    # print("k: %f" % ((lefty - righty) / (0 - cols + 1)))
    # print("degree: %f" % angle)
    return angle


def filter_for_ir(data_list):
    """

    :param data_list:
    :param np_ir:
    :param last_data_list: [[data1],
                             [data2],
                             ... ...
                             [data-n]}
    :return:
    """
    n_layer = len(data_list)
    np_sum = np.array(np.zeros(data_list[0].shape))
    for item in data_list:
        np_sum += item
    # print("result:", np_sum / n_layer)
    # print("inner",n_layer)
    return (np_sum / n_layer).reshape((1, -1))


def get_foot_ankle(cnt):
    point = get_convexity_point(cnt)
    ankle = [np.array(point[0][0]).reshape((1, 2)), np.array(point[1][0]).reshape((1, 2))]
    hull = cv.convexHull(cnt)
    foot = []
    y = bigger_y(ankle)
    for i in hull:
        if i[0][1] > y:
            foot.append(i)
    foot.append(ankle[0])
    foot.append(ankle[1])
    return np.array(foot)


def draw_hist(np_ir):
    histr = cv.calcHist([np_ir], [0], None, [256], [0, 256])
    plt.plot(histr, color="r")
    plt.xlim([0, 256])
    plt.show()
    return np_ir


def image_processing_mean_filter(np_ir, kernel_num=2):
    kernel = np.ones((kernel_num, kernel_num), np.float) / (kernel_num ** 2)
    res = cv.filter2D(np_ir, -1, kernel)
    return res


def image_processing_contrast_brightness(img, alpha, beta):
    blank = np.zeros(img.shape, img.dtype)
    # dst = alpha * img + beta * blank
    dst = cv.addWeighted(img, alpha, blank, 1 - alpha, beta)
    return dst


def show_temperature(temperature):
    print()
    for i in range(24):
        t = temperature[i]
        for j in range(32):
            print("%.2f" % t[j], end=" ")
        print()
    # print(temperature[0])

    pass


def detect_is_foot(temperature, temp_threshold=20):
    """
    这个函数的目的是，通过判断超过某一温度的个数，来判断该图中是否含有“脚”
    :param temperature:
    :return:
    """
    num = 0
    for item in temperature:
        if item >= 19:
            num += 1
    # print("bigger than 20 ", num)
    if num > 40:
        # print("have foot")
        return True
    else:
        # print("No foot")
        return False


def k_means_detect(temperature):
    """
    这个函数的目的是，使用聚类算法（k-means）去判断原始的温度矩阵中，哪些点属于人的身体，那些点属于环境
    已有的知识分类：“身体”点温度 > 环境点温度

    btw，注意这里传入的参数是list，并不是转成图片数据后的结果，如果想要打印结果需要转成np.reshape
    :param temperature:温度的list
    :return: 返回list ， list（reslut）， falg（身体的结果0 or 1）
    """
    temp_np = np.array(temperature).reshape((len(temperature), 1))
    # print(temp_np.shape)
    result = KMeans(n_clusters=2).fit_predict(temp_np)

    # 以图片方式print result
    # result1 = np.array(result).reshape(24, 32)
    # show_temperature(result1)

    # 脚的平均温度值应该比空白地方高
    temp0 = 0.0
    num0 = 0
    temp1 = 0.0
    num1 = 0
    env_max0 = 0.0
    env_max1 = 0.0
    for i in range(len(temperature)):
        if result[i] == 0:
            if env_max0 < temperature[i]:
                env_max0 = temperature[i]
            temp0 += temperature[i]
            num0 += 1
        else:
            if env_max1 < temperature[i]:
                env_max1 = temperature[i]
            temp1 += temperature[i]
            num1 += 1
    temp0 = temp0 / num0
    temp1 = temp1 / num1

    if temp0 > temp1:
        return result, 0, env_max1
    else:
        return result, 1, env_max0


def filter_temperature(temp, kernel_num=3):
    # print(temp)
    temp = np.array(temp).reshape(24, 32)
    # show_temperature(temp)
    kernel = np.ones((kernel_num, kernel_num), np.float) / (kernel_num ** 2)
    res = cv.filter2D(temp, -1, kernel)
    # show_temperature(res)
    result = np.array(res).reshape(1, 24 * 32).tolist()[0]
    return result


def filter_high_temperature(temp):
    # print(temp)
    high = [0, -1, 0, -1, 4, -1, 0, -1, 0]
    high_np = np.array(high).reshape(3, 3)
    print(high_np)
    temp = np.array(temp).reshape(24, 32)
    # show_temperature(temp)
    res = cv.filter2D(temp, -1, kernel=high_np)
    show_temperature(res)
    result = np.array(res).reshape(1, 24 * 32).tolist()[0]
    return result
