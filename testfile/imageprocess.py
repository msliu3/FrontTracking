#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   imageprocess.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/10/29 17:55   msliu      1.0      

@Description
------------
None
"""

import cv2 as cv
import numpy as np

img = cv.imread('../resource/temp.jpg', 1)
img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(img_gray, 127, 255, 0)
contours, hierarchy = cv.findContours(thresh, 2, 1)
cv.drawContours(img, contours, -1, (255, 0, 0), 3)
cnt = contours[0]
hull = cv.convexHull(cnt, returnPoints=False)
defects = cv.convexityDefects(cnt, hull)
for i in range(defects.shape[0]):
    s, e, f, d = defects[i, 0]
    start = tuple(cnt[s][0])
    end = tuple(cnt[e][0])
    far = tuple(cnt[f][0])
    cv.line(img, start, end, [0, 255, 0], 2)
    cv.circle(img, far, 5, [0, 0, 255], -1)
cv.imshow('img', img)
cv.waitKey(0)
cv.destroyAllWindows()
