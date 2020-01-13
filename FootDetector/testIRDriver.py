#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testIRDriver.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/28 19:55   msliu      1.0      

@Description
------------
None
"""

# import lib
import threading
import numpy as np

from FootDetector.DemonstrationProcess import DemonProcess
from DigitalDriver.ControlDriver import ControlDriver
dp = DemonProcess()
cd = ControlDriver()
p2 = threading.Thread(target=cd.control_part, args=())
p2.start()
head = []
data = []
filter_data = []
rest_num = 5
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
        if len(data) == rest_num:
            temp, ir_np, foot = dp.demonstrate_data(data[rest_num - 1], filter_data,
                                                    filter_num=2)  # ,zoom_filter=Image.HAMMING
            # ir_np = pf.draw_hist(ir_np)
            if foot:
                # ir_np = pf.image_processing_mean_filter(ir_np, kernel_num=5)
                # pf.show_temperature(temp)
                # ir_np = pf.image_processing_contrast_brightness(ir_np, 1.6, -0.8)
                ir_np, contours = dp.binary_image(np.array(ir_np))
                dp.find_foot_ankle(ir_np, contours)
                omega = dp.foot.Rotate_in_place(0.01)
                print(omega)
                cd.omega = omega
                if dp.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
                    break

            # ir_np, contours = dp.binary_image(np.array(ir_np))
            # dp.find_foot_ankle(ir_np, contours)
            # if dp.demo_record(ir_np) == -1:  # , 'continuous' , mode='frame-by-frame'
            #     break

            data.pop(rest_num - 1)
