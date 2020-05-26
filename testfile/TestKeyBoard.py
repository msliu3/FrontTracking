#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   TestKeyBoard.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/15 22:44   msliu      1.0         None
"""

import pygame
import os
from pygame.locals import *

WINDOW_W, WINDOW_H = 640, 480  # 窗体尺寸
FPS = 50  # 帧率，即每秒刷新多少次
g = 9.8 * 100  # 重力加速度（我们用的单位是像素每二次方秒）
pygame.init()  # 初始化pygame
# 设置窗口出现的位置
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (200, 100)
# 创建一个窗口
screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.DOUBLEBUF, 32)
# 设置窗口标题
pygame.display.set_caption("hello,world!")
# 创建时钟对象 (可以控制游戏循环频率)
clock = pygame.time.Clock()

pygame.init()


def method1():
    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                exit()
            if event.type == KEYDOWN:  # KEYDOWN 按键被按下
                if event.key == K_ESCAPE:
                    print('你按下了Esc键，准备退出')
                    exit()
                if event.key == K_LEFT or event.key == K_a:
                    # K_LEFT：左方向键
                    # K_a：A键
                    print('向左移动')
                if event.key in [K_RIGHT, K_d]:
                    print('向右运动')
                if event.key == K_SPACE:
                    print('按下了空格键')
            elif event.type == KEYUP:  # KEYUP 按键被松开
                if event.key in [K_LEFT, K_a]:
                    print('停止向左移动')
    pass


def method2():
    while True:
        key_pressed = pygame.key.get_pressed()
        if key_pressed[pygame.K_RIGHT] == 1:
            print("向右移动")
    pass

