"""
    SSL _ turning
"""
import math
import time


def SSLturning(cd, angle):
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
        """Zhuzhi version"""
        # while 1:
        #     if (cd.position[2] * expectedTHETA) > 0:
        #         break

        # if (cd.position[2] * expectedTHETA) >= 0 and rad > 0:
        #     while 1:
        #         if abs(cd.position[2] - expectedTHETA) <= 0.2:
        #             cd.omega = 0
        #             time.sleep(time_sleep_value)
        #             print('reached')
        #             break
        # elif (cd.position[2] * expectedTHETA) >= 0 and rad < 0:
        #     while 1:
        #         if abs(expectedTHETA - cd.position[2]) <= 0.2:
        #             cd.omega = 0
        #             time.sleep(time_sleep_value)
        #             print('reached')
        #             break

        """Chongyu's version"""
        time_out_flag = 0
        threshold = 700
        a = time.time()
        print("before:",cd.position[2], expectedTHETA)
        # while 1:
        #     if (cd.position[2] * expectedTHETA) > 0 or time_out_flag > threshold:
        #         if time_out_flag <= threshold:
        #             print("Next step")
        #             pass
        #         else:
        #             b = time.time()
        #             print("delay! :",b-a)
        #         break
        #     else:
        #         time_out_flag += 1
        #         time.sleep(0.01)
        # print("After:",cd.position[2], expectedTHETA)

        time_out_flag = 0
        a = time.time()
        # if (cd.position[2] * expectedTHETA) >= 0:
        #     while 1:
        #         if abs(cd.position[2] - expectedTHETA) <= 0.2 or time_out_flag > threshold:
        #             if time_out_flag <= threshold:
        #                 print("Next step")
        #                 pass
        #             else:
        #                 b = time.time()
        #                 print("delay! :", b - a)
        #             cd.omega = 0
        #             time.sleep(time_sleep_value)
        #             print('reached')
        #             break
        #         else:
        #             time_out_flag += 1
        #             time.sleep(0.01)
        # else:
        #     print('false')
        #     pass
        while 1:
            if abs(cd.position[2] - expectedTHETA) <= 0.2 or time_out_flag > threshold:
                if time_out_flag <= threshold:
                    print("Next step")
                    pass
                else:
                    b = time.time()
                    print("delay! :", b - a)
                cd.omega = 0
                time.sleep(time_sleep_value)
                print('reached')
                break
            else:
                time_out_flag += 1
                time.sleep(0.01)

    else:
        pass

    cd.omega = 0
    time.sleep(0.1)
    # print('final position: ', cd.position[2])
