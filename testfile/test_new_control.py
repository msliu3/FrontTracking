import os
import sys
import time
import threading
import DigitalDriver.ControlandOdometryDriver as CD
import Control.PositionControl2 as PC
import Control.PositionControl as PC1

def loop(pc, control, flag):
    while True:
        pc.top_decision(control)
        if flag == 0:
            pc.set_expect(0, 0)
            break


def loop2(control, pc):

    time_snap = 0.5
    print("speed:", pc.speed, "omega:", pc.omega, "radius", pc.radius, "time:", time.time())
    pc.set_expect(0, 0)
    time.sleep(time_snap)

    print("speed:", pc.speed, "omega:", pc.omega, "radius", pc.radius, "time:", time.time())
    pc.set_expect(1, 1)
    time.sleep(time_snap)

    print("speed:", pc.speed, "omega:", pc.omega, "radius", pc.radius, "time:", time.time())
    pc.set_expect(0, 0)
    time.sleep(time_snap)

    os._exit(1)
    # pc.set_expect(-1, 0)
    # time.sleep(2)
    #
    # pc.set_expect(0, 0)
    # time.sleep(2)


def loop_1(control, pc, bf, turn):
    if pc.action_over:
        if bf:
            pc.action_forward_back(control)
        elif turn:
            pc.action_forward_and_turning(control)


def loop_1_2(control, pc):

    pc.set_expect(0.1, 1)
    loop_1(control, pc, 1, 0)

    pc.set_expect(0.1, 40)
    loop_1(control, pc, 0, 1)

    pc.set_expect(0.1, -50)
    loop_1(control, pc, 0, 1)

    pc.set_exptpect(-0.1, 0)
    loop_1(control, pc, 1, 0)




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
    pc1 = PC1.PositionControl()
    flag_stop = 1
    p1 = threading.Thread(target=loop, args=(pc, cd, flag_stop))
    p1.start()
    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    # p3 = threading.Thread(target=loop_stop(), args=())
    # p3.start()
    # p3 = threading.Thread(target=loop_1_2, args=(cd,pc1))
    # p3.start()

    p2 = threading.Thread(target=loop2, args=(cd, pc))
    p2.start()