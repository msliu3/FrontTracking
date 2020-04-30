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


def loop(event):
    while True:
        time.sleep(1)
        end = input("end?")
        if end == "":
            print("------------------------------------------------")
            event.set()


def loop2(ss):
    while True:
        ss.read_softskin_data(0)
        time.sleep(0.3)


if __name__ == '__main__':
    ss = SS.SoftSkin()
    cd = CD.ControlDriver()
    ss.build_base_line_data()
    event = threading.Event()
    event.clear()
    p1 = threading.Thread(target=loop, args=(event,))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(ss,))
    p2.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    """检测调整"""
    while True:
        ss.adjust_direction(cd, using=False)

    """检测解锁"""
    # ss.locking = True
    # ss.unlock()
    # ss.is_locked = True
    # ss.brake_control(ss.locking)
    # time.sleep(1)

    """检测异常"""

    # print("start\n")
    # ss.is_locked = False
    # ss.detect_accident(using=True)
    # ss.brake_control(ss.locking)
    # print("detected!\n")
    # time.sleep(1.5)

    """上锁与解锁"""

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

