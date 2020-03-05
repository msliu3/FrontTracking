from DigitalDriver import ControlandOdometryDriver
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