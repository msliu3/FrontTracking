#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@File    :   testKeyboradpynput.py    
@Contact :   liumingshanneo@163.com

@Modify Time      @Author    @Version
------------      -------    --------
2019/12/5 21:24   msliu      1.0      

@Description
------------
None
"""

from pynput import keyboard, mouse


def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))


def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def on_move(x, y):
    print('Pointer moved to {0}'.format(
        (x, y)))


def on_click(x, y, button, pressed):
    print('{0} at {1}'.format(
        'Pressed' if pressed else 'Released',
        (x, y)))
    if not pressed:
        # Stop listener
        return False


def on_scroll(x, y, dx, dy):
    print('Scrolled {0} at {1}'.format(
        'down' if dy < 0 else 'up',
        (x, y)))


# Collect events until released

keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
mouse_listener = mouse.Listener(on_click=on_click, on_move=on_move, on_scroll=on_scroll)
lst = [keyboard_listener, mouse_listener]

for t in lst:
    t.start()

for t in lst:
    t.join()
