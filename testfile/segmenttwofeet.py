#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   segmenttwofeet.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/28 21:55   msliu      1.0      

@Description
------------
None
"""

import numpy as np
import cv2 as cv

import ProcessFunc as pf

image, contours = pf.get_contours('../resource/enlarge-original.jpg', 97)
cv.drawContours(image, contours, -1, (255, 0, 0), 3)
cv.imshow('img-gray', image)

sorted_point = sorted(contours, key=lambda x: len(x), reverse=True)
for c in contours:
    print("length:", len(c))

for c in sorted_point:
    print("length:", len(c))

cnt = contours[1]
# print(cv.contourArea(cnt))
# print(image.shape[0])
foot = []
foot = pf.select_contours(image, contours)
print(len(foot))
for item in foot:
    print("after", len(item))
    pf.draw_normal_rectangle(image, item)
    pf.draw_min_rectangle(image, item)

# for item in contours:
#     print(len(item))

# point = pf.get_convexity_point(cnt)
# left, right = pf.segmentation_two_feet(image, point)
# print(left)
# print(pf.bigger_y(right))


# foot = []
# for a in hull:
#     if a[0][1] > 100:
#         foot.append(a)
# # print(foot)
# foot.append(np.array(sorted_point[0][0]).reshape((1, 2)))
# foot.append(np.array(sorted_point[1][0]).reshape((1, 2)))
# print(foot)
# cv.imshow("img-contour",image)
#
# left_foot = np.array(img2[sorted_point[0][0][1] + 50:260, 0:sorted_point[0][0][0] + 15])
# cv.imwrite("../resource/segment.jpg", left_foot)
# # sorted_point[0][0][0]
# img_gray_left_foot = cv.cvtColor(left_foot, cv.COLOR_BGR2GRAY)
# ret_f, thresh_f = cv.threshold(img_gray_left_foot, 97, 255, 0)
# contours_f, hierarchy = cv.findContours(thresh_f, 2, 1)
# cv.drawContours(left_foot, contours_f[0], -1, (0, 255, 0), 3)
#
# hull_f = cv.convexHull(contours_f[0], returnPoints=False)
# defects_f = cv.convexityDefects(contours_f[0], hull_f)
# for i in range(defects_f.shape[0]):
#     s, e, f, d = defects_f[i, 0]
#     # two[tuple(cnt[f][0])] = d
#     far = tuple(cnt[f][0])
#     cv.circle(left_foot, far, 5, [255, 0, 0], -1)
# cv.imshow('left_foot', left_foot)
# hull = cv.convexHull(cnt)
# x, y, w, h = cv.boundingRect(np.array(foot))
# cv.rectangle(img2, (x, y), (x + w, y + h), (0, 0, 255), 2)
#
# rect = cv.minAreaRect(np.array(foot))
# box = cv.boxPoints(rect)
# box = np.int0(box)
# cv.drawContours(img2, [box], 0, (0, 0, 255), 2)
cv.imshow('img', image)
cv.waitKey(0)
cv.destroyAllWindows()
