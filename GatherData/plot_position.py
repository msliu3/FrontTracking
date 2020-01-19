from matplotlib import pyplot as plt
import re
import os


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
    for item in data:
        a_X.append(item[1])
        a_Y.append(item[2])
        l_X.append(item[1]+item[5])
        l_Y.append(item[2]+item[6])
        r_X.append(item[1]+item[7])
        r_Y.append(item[2]+item[8])
    plt.figure(1)
    plt.plot(a_Y, a_X, ls="-.")
    plt.plot(l_Y,l_X)
    plt.plot(r_Y,r_X)
    plt.figure(2)
    plt.plot(a_Y, a_X, ls="-.")
    plt.show()
    print(float("-0.1"))
