import threading

from DigitalDriver import ControlandOdometryDriver as CD
import math
import time

def SSLturning(cd, angle):
    # cd: an instance of class ControlandOdometryDriver,  angle: angle to turn as in degree
    # angle = 0, 45, 90, 135, 180, 225, 270, 315

    cd.omega = 0    # stop the walker
    cd.speed = 0
    cd.radius = 0

    if angle >= 180:
        rad = (360-angle)/180 * math.pi
    else:
        rad = -angle/180*math.pi

    currentTHETA = cd.position[2]   #read current THETA∈(-π，π]
    expectedTHETA = currentTHETA + rad
    if expectedTHETA > math.pi:
        expectedTHETA -= 2 * math.pi
    elif expectedTHETA <= -math.pi:
        expectedTHETA += 2 * math.pi

    if not rad:
        if rad > 0:
            cd.omega = math.pi / 8
        else:
            cd.omega = -math.pi / 8
        cd.radius = 0
        cd.speed = 0
        time.sleep(0.5)
        #这里必须先让车转起来再进入下一步

        while 1:
            if (cd.position[2]*expectedTHETA) > 0:
                break

        if rad > 0:
            while 1:
                if cd.position[2] - expectedTHETA >= 0:
                    break
        else:
            while 1:
                if expectedTHETA - cd.position[2] >= 0:
                    break
        # stop moving
        cd.omega = 0
        time.sleep(0.5)
    else:
        pass

def SSLturning2(cd, angle):
    time_sleep_value = 0.05
    cd.speed = 0
    cd.omega = 0
    cd.radius = 0
    # cd: an instance of class ControlandOdometryDriver,  angle: angle to turn as in degree
    # angle = 0, 45, 90, 135, 180, 225, 270, 315
    if angle > 180:
        rad = (360 - angle) / 180 * math.pi
    else:
        rad = -angle / 180 * math.pi

    currentTHETA = cd.position[2]  # read current THETA∈(-π，π]
    expectedTHETA = currentTHETA + rad

    if expectedTHETA > math.pi:
        expectedTHETA -= 2 * math.pi
    elif expectedTHETA <= -math.pi:
        expectedTHETA += 2 * math.pi

    # print('rad: ', rad, ';  Current theta: ', currentTHETA, '; Expected theta: ', expectedTHETA)

    if rad != 0:
        if rad > 0:
            cd.omega = math.pi / 6
        else:
            cd.omega = - math.pi / 6
        cd.radius = 0
        cd.speed = 0
        time.sleep(time_sleep_value)
        # print('start moving...')

        while 1:
            if (cd.position[2] * expectedTHETA) > 0:
                break

        if (cd.position[2] * expectedTHETA) >= 0 and rad > 0:
            while 1:
                if abs(cd.position[2] - expectedTHETA) <= 0.2:
                    cd.omega = 0
                    time.sleep(time_sleep_value)
                    # print('reached')
                    break
        elif (cd.position[2] * expectedTHETA) >= 0 and rad < 0:
            while 1:
                if abs(expectedTHETA - cd.position[2]) <= 0.2:
                    cd.omega = 0
                    time.sleep(time_sleep_value)
                    # print('reached')
                    break
        else:
            print('false')
            pass
    else:
        pass

    cd.omega = 0
    # time.sleep(0.1)

def loop(control):
    while True:
        time.sleep(1)
        temp = input("angle")
        if temp != "end":
            print(int(temp))
            SSLturning2(control,int(temp))

        if temp == "end":
            print("end!!!")
            control.flag_end = 1
            break
if __name__ == '__main__':
    cd = CD.ControlDriver()
    p1 = threading.Thread(target=loop, args=(cd,))
    p2 = threading.Thread(target=cd.control_part, args=())
    print("hehe")
    p2.start()
    p1.start()
