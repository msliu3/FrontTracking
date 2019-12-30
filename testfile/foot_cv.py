#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   foot_cv.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/28 16:22   msliu      1.0      

@Description
------------
这个是加入脚踝的两个点的测试文件
"""

import numpy as np
import cv2 as cv
from FootDetector import ProcessFunc as pf

img = cv.imread('../resource/enlarge-original.jpg', 0)
img2 = cv.imread('../resource/segment.jpg', 1)
# enlarge-original.jpg

# img_gray = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(img, 127, 255, 0)

contours, hierarchy = cv.findContours(thresh, 2, 1)

# 二值图分割情况
cv.drawContours(img, contours, -1, (255, 0, 0), 3)
cv.imshow('img2', img)

for cnt in contours:
    hull = cv.convexHull(cnt, returnPoints=False)
    defects = cv.convexityDefects(cnt, hull)
    two = {}
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        two[tuple(cnt[f][0])] = d
        far = tuple(cnt[f][0])
        cv.circle(img2, far, 5, [0, 0, 255], -1)
    sorted_point = sorted(two.items(), key=lambda x: x[1], reverse=True)
    # print(sorted_point)
    hull = cv.convexHull(cnt)
    foot = []
    for a in hull:
        if a[0][1] > 100:
            foot.append(a)
    # print(foot)
    foot.append(np.array(sorted_point[0][0]).reshape((1, 2)))
    foot.append(np.array(sorted_point[1][0]).reshape((1, 2)))
    # print(foot)
    x, y, w, h = cv.boundingRect(np.array(foot))
    cv.rectangle(img2, (x, y), (x + w, y + h), (0, 0, 255), 2)

    rect = cv.minAreaRect(np.array(foot))
    for item in rect:
        print(item)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    cv.drawContours(img, [box], 0, (0, 0, 255), 2)
    l, r = pf.draw_min_line(img, np.array(foot))
    print(l,r)
cv.imshow('img', img)
cv.waitKey(0)
cv.destroyAllWindows()
