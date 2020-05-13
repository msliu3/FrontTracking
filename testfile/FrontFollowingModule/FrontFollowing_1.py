#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   FrontFollowing_1.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2020/5/11 18:44   msliu      1.0      

@Description
------------
组成模块的第一个版本2020-5-11

组成一个类：
开启和关闭接口
接收controldriver
"""

import os
import sys

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + ".."+os.path.sep + "..")
print(father_path)
sys.path.append(father_path)
# 导入路径，确保ros打开能开启相应文件

import FootDetector.DemonstrationProcess as DP
import Control.DeepLearning_DetectCase as DL
import Control.MatchCase_NN as MC
import DigitalDriver.ControlDriver as CD
import Control.PositionControl as PC

import time
import numpy as np
from multiprocessing import Process
import threading
import multiprocessing


class FrontFollowing(Process):
    def __init__(self, ControlDriver):
        # 继承父类
        Process.__init__(self)

        # self.control_driver = CD.ControlDriver()
        self.control_driver = ControlDriver

        # 开启IR camera，开启一条进程处理数据
        self.ir_camera = DP.DemonProcess()

        # 打开一个消息队列用于
        self.queue = multiprocessing.Queue()

        # self.matcher = MC.MatchCase(foot=self.ir_camera.foot)
        # print("matcher id",id(self.matcher))

        self.position_control = PC.PositionControl()

        # 事件控制：控制机器人是否执行移动逻辑
        self.event_action_loop = threading.Event()



        self.flag_stop = 1
        pass

    def stop_front_following(self):
        self.event_action_loop.wait()
        pass

    def resume_front_following(self):
        self.event_action_loop.clear()
        pass

    def run(self) -> None:
        matcher = MC.MatchCase(foot=self.ir_camera.foot)
        while True:
            time.sleep(1)
            print(matcher.leg.left_leg_x,matcher.leg.right_leg_x,id(matcher.leg))
        # thread_ir = multiprocessing.Process(target=self.ir_camera.start_Demon_for_DL, args=(self.queue,))
        # thread_ir.start()
        # p1 = threading.Thread(target=self.action_loop, args=())
        # p1.start()
        # p2 = threading.Thread(target=self.detecting_loop, args=())  # nn_model,
        # p2.start()

    def action_loop(self):
        while True:
            print("------action_loop started------")
            self.event_action_loop.wait()

            # position_control负责计算目标位置，以及如何移动过去
            if self.position_control.action_over:
                if self.matcher.back or self.matcher.forward:
                    # 负责前进和后退的控制

                    # print("back_forward", position_control.expect_x, position_control.expect_theta)
                    self.position_control.action_forward_back(self.control_driver)
                    self.matcher.clear_case()

                elif self.matcher.turning:
                    # 负责转弯的控制

                    # print("turn")
                    self.position_control.action_forward_and_turning(self.control_driver)
                    self.matcher.clear_case()
            self.event_action_loop.clear()
            print("------action_loop end------")

    def detecting_loop(self):
        nn_model = DL.DeepLearningDetectCase(model_name="dnn1.ckpt")
        while True:
            time.sleep(0.05)
            if not self.queue.empty():
                print("------detecting_loop started------")
                # print("into queue")
                # matcher.clear_foot_and_leg()
                temps = self.queue.get(block=False)
                print("matcher id2", id(self.matcher))
                print("leg:",
                      self.matcher.leg.left_leg_x,
                      self.matcher.leg.left_leg_y,
                      self.matcher.leg.right_leg_x,
                      self.matcher.leg.right_leg_y
                      )
                leg_temp = np.array(
                    [self.matcher.leg.left_leg_x, self.matcher.leg.left_leg_y, self.matcher.leg.right_leg_x,
                     self.matcher.leg.right_leg_y]).reshape([1, 4])

                ir_np = np.array(temps).reshape([32 * 24, nn_model.sample_num])
                leg_np = np.array(leg_temp).reshape([4, nn_model.sample_num])
                # print(leg_np[0])
                # print("times: ",sum)
                # result = 2
                result = nn_model.print_predict_result(ir_np, leg_np)
                self.matcher.detect_front_and_back_foot()
                x, theta = self.matcher.detect_case(result)
                print(x, theta)
                self.position_control.set_expect(x, theta)
                self.event_action_loop.set()
                self.matcher.clear_expect()
if __name__ == '__main__':
    control_driver = CD.ControlDriver()
    control_driver.start()
    ff = FrontFollowing(control_driver)
    ff.start()
