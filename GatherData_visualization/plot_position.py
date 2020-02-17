from matplotlib import pyplot as plt
import re
import os
import math
import numpy as np

"""
0   time.time(),
1   cd.odo.getROS_XYTHETA()[0],
2   cd.odo.getROS_XYTHETA()[1],
3   cd.odo.THETA,
    # cd.odo.getROS_XTTHETA()[2],
4   cd.odo.get_dxdydtheta()[0],
5   cd.odo.get_dxdydtheta()[1],
6   leg.left_leg_x,
7   leg.left_leg_y,
8   leg.right_leg_x,
9   leg.right_leg_y,
10  IMU.imu_human
"""


def read_file_to_list(file):
    list_data = []
    with open(file) as f:
        line = f.readline()
        line = line.rstrip('\n')
        line_data = re.findall(r"(.\d+\.\d+)", line)
        line_data = list(map(strToFloat, line_data))
        print(line_data)
        list_data.append(line_data)
        while line:
            line = f.readline()
            line = line.rstrip('\n')
            if line != "":
                line_data = re.findall(r"(.\d+\.\d+)", line)
                line_data = list(map(strToFloat, line_data))
                print(line_data)
                list_data.append(line_data)
    return list_data


def coordinate_transformation(theta, x, y):
    theta_temp = theta
    # print(theta)
    trans = np.array([[math.cos(theta_temp), math.sin(theta_temp)],
                      [-math.sin(theta_temp), math.cos(theta_temp)]])
    result = np.dot(trans, np.array([[x], [y]]))
    print("-----------------log information-------------------------")
    print("degree:", math.degrees(theta_temp), "rad:", theta_temp)
    print(x, result[0])
    print(y, result[1])
    return result


def difference_list(list_data):
    dif_list = []
    for i in range(1, len(list_data) - 1):
        if list_data[i] * list_data[i - 1] < 0 and abs(list_data[i]) + abs(list_data[i - 1]) > 180:
            # print("into", list_data[i-1], list_data[i],end=" ")
            dif = (list_data[i - 1] / abs(list_data[i - 1])) * 180 - list_data[i - 1]
            dif += list_data[i] - (list_data[i] / abs(list_data[i])) * 180
            # print(dif)
        else:
            dif = list_data[i] - list_data[i - 1]
        dif_list.append(dif)
    return dif_list


def dif_two_theta(first, second):
    if first * second < 0 and abs(first) + abs(second) > 180:
        # print("into", list_data[i-1], list_data[i],end=" ")
        dif = (second / abs(second)) * 180 - second
        dif += first - (first / abs(first)) * 180
        # print(dif)
    else:
        dif = first - second
    return dif


def strToFloat(number):
    try:
        return float(number)
    except:
        return number


def draw_pic(path, num):
    data = read_file_to_list(path)
    a_X = []
    a_Y = []
    l_X = []
    l_Y = []
    r_X = []
    r_Y = []
    imu = []
    robot_theta = []
    dif_imu_rob = []
    dif_odo_imu = []
    lr_X = []
    lr_Y = []
    rr_X = []
    rr_Y = []
    for item in data:
        # 在笛卡尔坐标中  —>为正X 向前为正Y
        abs_X = -item[2]
        abs_Y = item[1]
        print("coordinate: ", abs_X, abs_Y)
        # 因为第一次变换坐标系，

        a_X.append(-item[2])
        a_Y.append(-item[1])
        result = coordinate_transformation(item[3], -item[7], item[6])

        l_X.append(result[0] - item[2])
        l_Y.append(result[1] - item[1])
        result = coordinate_transformation(item[3], -item[9], item[8])
        r_X.append(result[0] - item[2])
        r_Y.append(result[1] - item[1])
        lr_X.append(item[6])
        lr_Y.append(item[7])
        rr_X.append(item[8])
        rr_Y.append(item[9])
        imu.append(item[10])
        robot_theta.append(math.degrees(item[3]))
        dif = dif_two_theta(item[10], math.degrees(item[3]))
        dif_imu_rob.append(dif)
        dif = dif_two_theta(math.degrees(item[3]), item[10])
        dif_odo_imu.append(dif)
    plt.figure(num + 1)
    plt.plot(a_X, a_Y, ls="-.")
    plt.scatter(l_X, l_Y, ls="-", color="r")
    plt.scatter(r_X, r_Y, ls="-", color="b")
    # temp = input("next")
    # if not str(temp) == "0":
    #     break
    # else:
    #     plt.show()
    # plt.plot(l_Y,l_X)
    # plt.plot(r_Y,r_X)
    plt.figure(num + 2)
    # plt.plot(robot_theta, color="r")
    plt.plot(difference_list(robot_theta), color="b")

    # plt.plot(imu, color="b")
    # plt.figure(3)
    # plt.plot(dif_imu_rob, color="r")
    # plt.plot(difference_list(imu), color="g")
    # plt.plot(dif_odo_imu)
    # plt.figure(4)
    # plt.plot(lr_Y, lr_X, color="g")
    # plt.plot(rr_Y, rr_X, color="b")
    return difference_list(robot_theta)


if __name__ == '__main__':
    pwd = os.path.abspath(os.path.abspath(__file__))
    father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
    data_path_0 = father_path + os.path.sep + "resource" + os.path.sep + "1-20_19.34直行" + os.path.sep + "leg_odo.txt"
    data_path_1 = father_path + os.path.sep + "resource" + os.path.sep + "1-20_19.38右转" + os.path.sep + "leg_odo.txt"
    data_path_2 = father_path + os.path.sep + "resource" + os.path.sep + "1-20_19.42左转" + os.path.sep + "leg_odo.txt"
    data_path_3 = father_path + os.path.sep + "resource" + os.path.sep + "1-20_19.49环路" + os.path.sep + "leg_odo.txt"
    print(data_path_0)
    a = draw_pic(data_path_0, 0)
    b = draw_pic(data_path_1, 2)
    c = draw_pic(data_path_2, 5)
    d = draw_pic(data_path_3,10)
    plt.figure(100)
    plt.plot(a)
    plt.plot(b, color="r")
    plt.plot(c, color="b")
    plt.show()
    print(math.degrees(2.118172), math.degrees(2.010722))
