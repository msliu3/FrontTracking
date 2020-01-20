from matplotlib import pyplot as plt
import re
import os
import numpy as np
import math


def read_file_to_list(file):
    list_data = []
    with open(file) as f:
        line = f.readline()
        line = line.rstrip('\n')
        line_data = re.findall(r"(\d+\.\d+)", line)
        line_data = list(map(strToFloat, line_data))
        print(line_data)
        list_data.append(line_data)
        while line:
            line = f.readline()
            line = line.rstrip('\n')
            if line != "":
                line_data = re.findall(r"(.\d+\.\d+)", line)
                line_data = list(map(strToFloat, line_data))
                list_data.append(line_data)
    return list_data


def strToFloat(number):
    try:
        return float(number)
    except:
        return number


if __name__ == '__main__':
    pwd = os.path.abspath(os.path.abspath(__file__))
    father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
    data_path = father_path + os.path.sep + "resource" + os.path.sep + "leg_odo.txt"
    data = read_file_to_list(data_path)
    a_X = []
    a_Y = []
    l_X = []
    l_Y = []
    r_X = []
    r_Y = []
    trans_matrix = np.ones((2, 2))
    for item in data:
        a_X.append(item[1])
        a_Y.append(item[2])
        # print("%f\n"%(item[3]))
        # print("%f\t%f\t%f\t%f\n" % (item[5],item[6],item[7],item[8]))
        trans_matrix = [[math.cos(item[3]), math.sin(item[3])],
                        [math.sin(-item[3]), math.cos(item[3])]]
        trans_matrix_inv = np.linalg.inv(trans_matrix)
        new_item = np.ones((2, 2))
        new_item[0][0] = item[5]
        new_item[1][0] = item[6]
        new_item[0][1] = item[7]
        new_item[1][1] = item[8]
        new_xy_item = np.dot(trans_matrix_inv, new_item)
        l_X.append(item[1]+new_xy_item[0][0])
        l_Y.append(item[2]+new_xy_item[1][0])
        r_X.append(item[1]+new_xy_item[0][1])
        r_Y.append(item[2]+new_xy_item[1][1])
    plt.figure(1)
    plt.plot(a_Y, a_X, ls="-.")
    plt.plot(l_Y,l_X)
    plt.plot(r_Y,r_X)
    plt.figure(2)
    plt.plot(a_Y, a_X, ls="-.")
    plt.show()
    print(float("-0.1"))
