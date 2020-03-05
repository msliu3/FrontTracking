import os
import sys
import time
import threading
import DigitalDriver.ControlandOdometryDriver as CD
import Control.PositionControl2 as PC

def loop(pc, flag):
    while True:
        time.sleep(0.5)
        expect_x = float(input("input:x"))
        expect_theta = float(input("input:theta"))
        pc.set_expect(expect_x, expect_theta)
        if flag == 0:
            pc.set_expect(0, 0)
            break


def loop2(control, pc):
    # while True:
    #     pc.top_decision(control)
    #     time.sleep(1.5)
    time_snap = 1
    pc.set_expect(0, 0)
    pc.top_decision(control)
    time.sleep(time_snap)

    # pc.set_expect(1, 10)
    # pc.top_decision(control)
    # time.sleep(time_snap*4)

    pc.set_expect(1, 90)
    pc.top_decision(control)
    print(time.time())
    time.sleep(time_snap)

    pc.set_expect(1, 40)
    pc.top_decision(control)
    print(time.time())
    time.sleep(time_snap*3)

    pc.set_expect(0, 0)
    pc.top_decision(control)
    time.sleep(time_snap)



def loop_stop():
    global flag_stop
    while True:
        end = input()
        if end == "":
            flag_stop = 0
            print("------------------------------------------------")
            print("------------------------------------------------")
            break


if __name__ == '__main__':
    cd = CD.ControlDriver()
    pc = PC.PositionControl2()
    flag_stop = 1
    # p1 = threading.Thread(target=loop, args=(pc, flag_stop))
    # p1.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()
    p2 = threading.Thread(target=loop2, args=(cd, pc))
    p2.start()
    # p3 = threading.Thread(target=loop_stop(), args=())
    # p3.start()
