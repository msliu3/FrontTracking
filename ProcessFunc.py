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


def get_contours(filename, threshhold=97):
    if type(filename) is type(''):
        img = cv.imread(filename, 1)
    else:
        img = filename
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(img_gray, threshhold, 255, 0)
    contours, hierarchy = cv.findContours(thresh, 2, 1)
    return img, contours


def select_contours(img, contours):
    cnt_list = []
    image_area = img.shape[0] * img.shape[1]
    for item in contours:
        area = cv.contourArea(item)
        rate = area / image_area
        print("area:", area, "rate", area / image_area)
        if rate > 0.8:
            cnt_list.append(area)
    if len(cnt_list) == 1:
        cnt_list = segmentation_two_feet(cnt_list)
    return cnt_list


def segmentation_two_feet(img, convex_point):
    segment_point_x = convex_point[0][0][0]
    segment_point_y = convex_point[0][0][1]
    left_foot = np.array(img[segment_point_y:img.shape[0], 0:segment_point_x])
    image, contours = get_contours(left_foot, threshhold=127)
    point = get_convexity_point(contours[0], image)
    segment_l = [np.array(point[0][0]).reshape((1, 2)), np.array(point[1][0]).reshape((1, 2))]
    hull = cv.convexHull(contours[0])
    left = []
    y = bigger_y(segment_l)
    for i in hull:
        if i[0][1] > y:
            left.append(i)
    left.append(segment_l[0])
    left.append(segment_l[1])

    right_foot = np.array(img[segment_point_y:img.shape[0], segment_point_x:img.shape[1]])
    image, contours = get_contours(right_foot, threshhold=127)
    # 这块有问题
    point = get_convexity_point(contours[1], image)
    segment_r = [np.array(point[0][0]).reshape((1, 2)), np.array(point[1][0]).reshape((1, 2))]
    hull = cv.convexHull(contours[1])
    right = []
    y = bigger_y(segment_r)
    for i in hull:
        if i[0][1] > y:
            right.append(i)
    right.append(segment_r[0])
    right.append(segment_r[1])
    for i in range(len(right)):
        right[i][0][0] += segment_point_x
    # cv.imwrite("../resource/temp.jpg", left_foot)
    # cv.drawContours(image, contours, -1, (255, 0, 0), 3)
    # cv.imwrite("../resource/temp-contours.jpg", left_foot)
    cv.imshow("temp", image)

    return np.array(left), np.array(right)


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
    print(sorted_point)
    return sorted_point


def draw_three(image, cnt_list):
    return


def draw_normal_rectangle(img, np_point, color=(0, 0, 255)):
    x, y, w, h = cv.boundingRect(np_point)
    cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
    return x, y


def draw_min_rectangle(img, np_point, color=(0, 0, 255)):
    rect = cv.minAreaRect(np_point)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, color, 2)
    return rect


def draw_min_line(img, np_point, color=(0, 0, 255)):
    rows, cols = img.shape[:2]
    [vx, vy, x, y] = cv.fitLine(np_point, cv.DIST_L2, 0, 0.01, 0.01)
    lefty = int((-x * vy / vx) + y)
    righty = int(((cols - x) * vy / vx) + y)
    cv.line(img, (cols - 1, righty), (0, lefty), (0, 255, 0), 2)
    return
