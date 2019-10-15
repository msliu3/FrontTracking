#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   TestKeyBoard.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version    @Desciption
------------      -------    --------    -----------
2019/10/15 22:44   msliu      1.0         None
"""

# import lib
import pygame
pygame.init()
while True:
    event=pygame.event.get()
    if event.type == pygame.KEYDOWN and event.key == pygame.K_RIGHT:
        print("HEHE")
