import os
import sys
import time
import numpy as np
import DigitalDriver.ControlandOdometryDriver as CD

pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)

import SoftSkin.SoftSkin as SS
import threading
import multiprocessing


def motor(cd,event):
    while True:
        event.wait()
        print("motor running")
        time.sleep(1)
        cd.omega = 0
        cd.speed = 0.1
        cd.radius = 0


def skin(ss,event,cd):
    while True:
        print("detecting stopping")
        ss.stop_ssl(1,cd,event)
        print("ssl stoped!")
        time.sleep(2)
        print("detecting locking")
        ss.lock(cd,1)
        time.sleep(2)
        print("detecting unlocking")
        ss.unlock()
        ss.brake_control()
        time.sleep(2)
        cd.stopMotor()
        # event.set()





if __name__ == '__main__':
    ss = SS.SoftSkin()
    cd = CD.ControlDriver()
    ss.build_base_line_data()
    event = threading.Event()
    event.set()
    thread_motor = threading.Thread(target=motor, args=(cd,event,))
    thread_skin = threading.Thread(target=skin,args=(ss,event,cd,))
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()
    # thread_motor.start()
    thread_skin.start()

    ss.is_locked = True
    ss.locking = False
    time.sleep(1)
    ss.brake_control()


    # cd.stopMotor()

    """检测调整"""
    # while True:
    #     ss.adjust_direction(cd, using=False)

    # """检测解锁"""
    # time.sleep(2)
    # print("time to sleep")
    #
    # # ss.locking = False
    # # ss.is_locked = True
    # # ss.brake_control(ss.locking)
    #
    # ss.locking = True
    # ss.is_locked = False
    # ss.brake_control(ss.locking)
    # ss.is_locked = True
    # ss.unlock()
    # ss.brake_control(ss.locking)


    """检测异常"""

    # print("start\n")
    # ss.is_locked = False
    # ss.detect_accident(using=True)
    # ss.brake_control(ss.locking)
    # print("detected!\n")
    # time.sleep(1.5)
    # time.sleep(0.5)
    # print("yes")
    # ss.locking = False
    # ss.is_locked = True
    # ss.brake_control(ss.locking)

    """终止SSL"""

    # print("start\n")
    # ss.is_locked = False
    # ss.stop_ssl(SSLrunning=True)
    # ss.brake_control(ss.locking)
    # print("detected!\n")
    # time.sleep(1.5)
    # time.sleep(0.5)
    # print("yes")
    # ss.locking = False
    # ss.is_locked = True
    # ss.brake_control(ss.locking)


    """
    记录数据用
    """
    # data_path = father_path + os.path.sep + "resource" + os.path.sep + "softskin.txt"
    # print(data_path)
    # with open(data_path, 'w') as file:
    #     while True:
    #         time.sleep(0.2)
    #         # print("%f"%cd.odo.Radius)
    #         if len(ss.raw_data) != len(ss.base_data):
    #             continue
    #         k = str(np.array(ss.raw_data) - np.array(ss.base_data))
    #
    #         k = k.strip("[")
    #         k = k.replace("]","\n")
    #         k = k.replace(",", " ")
    #         k = k.replace("  ", " ")
    #         k = k.replace("  ", " ")
    #         k = k.replace("  ", " ")
    #         k = k.replace("  ", " ")
    #         k = k.replace("  ", " ")
    #
    #         k = k.strip(" ")
    #         k = k.replace(" ", "\t")
    #         print(k)
    #         file.write(k)
    #
    #         if event.is_set():
    #             break
    #     file.close()

