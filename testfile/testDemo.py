#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import os
import sys
import time

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
print(father_path)
sys.path.append(father_path)

import threading
import DigitalDriver.ControlandOdometryDriver as CD
import multiprocessing
import SoftSkin.SoftSkin as SFSK
import testfile.ssl_loop as ssl
import testfile.FrontFollowingModule.FrontFollowing_1 as FrFol




def main_loop(skin, event_SSL, ffl, cd):
    while True:
        print("start again!")
        event_SSL.set()
        # SSL正在运行,检测Softskin是否有信号，需要停止SSL，停止方式把event_SSL堵塞
        SSLusing = True
        skin.adjust_direction(cd, event_SSL)
        skin.flag_read = 1
        time.sleep(10)
        print("Detecting unlock command")
        skin.flag_read = 0
        skin.unlock_new()
        skin.flag_read = 1
        time.sleep(3)
        skin.flag_read = 0
        while True:
            print("Front follow start!")
            ffl.resume_front_following()
            # FrontFollow 正在运行,检测Softskin是否有信号，需要停止SSL，停止方式把event_SSL堵塞
            Frontusing = True
            skin.detect_accident(cd, Frontusing)
            skin.flag_read = 1
            ffl.stop_front_following()
            time.sleep(5)
            # for i in range(175):
            #     skin.read_softskin_data(0)
            skin.flag_read = 0
            print("Detecting unlock command")
            skin.unlock_new()

def main_loop_skin_and_SSL(skin, event_SSL, cd):
    # skin.build_base_line_data()
    while True:
        print("start again!")
        event_SSL.set()
        # SSL正在运行,检测Softskin是否有信号，需要停止SSL，停止方式把event_SSL堵塞
        SSLusing = True
        skin.stop_ssl(SSLusing, cd, event_SSL)
        time.sleep(2)
        print("Detecting unlock command")
        skin.unlock()
        event_SSL.set()


def main_loop_skin_and_frontfollow(skin, ffl, cd):
    while True:
        print("start again!")
        ffl.resume_front_following()
        # FrontFollow 正在运行,检测Softskin是否有信号，需要停止SSL，停止方式把event_SSL堵塞
        Frontusing = True
        skin.detect_accident(cd, Frontusing)
        ffl.stop_front_following()
        time.sleep(2)
        for i in range(100):
            skin.read_softskin_data(0)
        print("Detecting unlock command")
        skin.unlock_new()



def skinloop(ss,event,cd):
    while True:
        event.set()
        print("detecting stopping")
        ss.stop_ssl(1,cd,event)
        print("ssl stoped!")
        time.sleep(2)
        event.clear()
        # print("detecting locking")
        # ss.lock(cd,1)
        # time.sleep(2)
        print("detecting unlocking")
        ss.unlock()
        # ss.brake_control()
        time.sleep(2)
        # cd.stopMotor()
        # event.set()

def testThread(event,cd):
    while True:
        event.wait()
        cd.speed = 0.0
        cd.omega = 0.3
        cd.radius = 0.0
        print("The test thread is running")
        time.sleep(2)


if __name__ == '__main__':

    cd = CD.ControlDriver()
    sound_local = ssl.SSL()

    event_SSL = threading.Event()
    event_SSL.set()


    skin = SFSK.SoftSkin()
    skin.build_base_line_data()

    thread_ssl = threading.Thread(target=sound_local.loop, args=(event_SSL, cd, 'test',))
    thread_ssl.start()

    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()
    ffl = FrFol.FrontFollowing(cd)
    ffl.start()
    print("Start!")
    time.sleep(0.4)
    ffl.stop_front_following()
    print("Stop!")

    # thread_skin_frontfollow = threading.Thread(target=main_loop_skin_and_frontfollow, args=(skin, ffl, cd,))
    # thread_skin_frontfollow.start()

    # thread_skin_ssl = threading.Thread(target=main_loop_skin_and_SSL, args=(skin, event_SSL, cd))
    # thread_skin_ssl.start()

    thread_main = threading.Thread(target=main_loop, args=(skin, event_SSL, ffl, cd,))
    thread_main.start()

    # thread_record = threading.Thread(target=skin.record, args=())
    # thread_record.start()

    # """测试skin用"""
    # event_test = threading.Event()
    # event_test.set()
    # thread_test = threading.Thread(target=testThread, args=(event_test,cd,))
    # thread_test_skin = threading.Thread(target=skinloop, args=(skin,event_test,cd,))
    # thread_test.start()
    # thread_test_skin.start()

    # 解锁用
    # skin.is_locked = True
    # skin.locking = False
    # time.sleep(1)
    # skin.brake_control()



