#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   opencv_MatchShapes.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/1/9 5:08   msliu      1.0      

@Description
------------
None
"""

# import lib
import os

import numpy as np
import cv2 as cv
import FootDetector.ProcessFunc as pf


def watershed(imgpath):
    img = cv.imread(imgpath)
    # cv.imshow('temp_img', img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret0, thresh0 = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

    kernel = np.ones((3, 3), np.uint8)
    opening = cv.morphologyEx(thresh0, cv.MORPH_OPEN, kernel, iterations=2)

    # 确定背景区域
    sure_bg = cv.dilate(opening, kernel, iterations=10)

    # 确定前景区域
    dist_transform = cv.distanceTransform(opening, cv.DIST_L2, 5)
    ret1, sure_fg = cv.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)

    # 查找未知区域
    sure_fg = np.uint8(sure_fg)
    unknown = cv.subtract(sure_bg, sure_fg)

    # 标记标签
    ret2, markers1 = cv.connectedComponents(sure_fg)
    markers = markers1 + 1
    markers[unknown == 255] = 0

    markers3 = cv.watershed(img, markers)
    img[markers3 == -1] = [0, 255, 0]
    return thresh0, sure_bg, sure_fg, img


def match_shape(img_path):
    img, contours = pf.get_contours(img_path, threshhold=127)
    print(len(contours))
    list_cnt = pf.sort_contours(img, contours)
    cnt1 = list_cnt[0]
    cnt2 = list_cnt[1]
    # bgr
    cv.drawContours(img, cnt1, -1, (255, 0, 0), 3)
    cv.drawContours(img, cnt2, -1, (0, 255, 0), 3)

    return cnt1, cnt2, img


if __name__ == '__main__':
    resource = os.path.abspath(
        os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".." + os.path.sep + "resource")
    imgpath = resource + os.path.sep + "one.png"
    imgpath1 = resource + os.path.sep + "one_big.png"
    imgpath2 = resource + os.path.sep + "one_small.png"
    imgpath3 = resource + os.path.sep + "foot.jpg"
    ret0_1, ret0_2, img = match_shape(imgpath)
    img1, contours1 = pf.get_contours(imgpath1, threshhold=127)
    cv.drawContours(img1, contours1[0], -1, (0, 255, 0), 3)
    img2, contours2 = pf.get_contours(imgpath2, threshhold=127)
    cv.drawContours(img2, contours2[0], -1, (0, 255, 0), 3)
    ret2_1, ret2_2, img3 = match_shape(imgpath3)
    # 3 > 1 > 2
    print(cv.matchShapes(ret0_1, contours1[0], 3, 0.0), "normal and big")
    print(cv.matchShapes(ret0_1, contours2[0], 3, 0.0),"normal and small")
    print(cv.matchShapes(contours1[0], contours2[0], 3, 0.0),"big and small")
    print(cv.matchShapes(ret0_1, ret2_1, 3, 0.0),"normal and left")
    print(cv.matchShapes(ret0_1, ret2_2, 3, 0.0), "normal and right")

    # print(cv.matchShapes(contours[0], ret3_2, 2, 0.0))
    # 2
    # 2.567657983551602
    # 0.8041806771640329
    # 1.0898127158094943
    # 3
    # 0.25462541363684293
    # 0.1209270464249637
    # 0.2510229765139863

    cv.imshow('normal', img)
    cv.imshow('big', img1)
    cv.imshow('small', img2)
    cv.imshow('left and right', img3)
    cv.waitKey(0)
    cv.destroyAllWindows()

    thresh0, sure_bg, sure_fg, img = watershed(imgpath2)
    cv.imshow('thresh0', thresh0)
    cv.imshow('sure_bg', sure_bg)
    cv.imshow('sure_fg', sure_fg)
    cv.imshow('result_img', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
