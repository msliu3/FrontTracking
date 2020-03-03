import numpy as np
import matplotlib.pyplot as plt
import math

path_dict = {
    "Walking with aid_FRONT": r'D:\HKU\Elderly Assistance\front_following_msliu\PatternRecognition\Walking with support\FRONT30.txt',
    "Walking with aid_LEFT": r'D:\HKU\Elderly Assistance\front_following_msliu\PatternRecognition\Walking with support\LEFT30.txt',
    "Walking with aid_RIGHT": r'D:\HKU\Elderly Assistance\front_following_msliu\PatternRecognition\Walking with support\RIGHT30.txt',
    "Sit down": r'D:\HKU\Elderly Assistance\front_following_msliu\PatternRecognition\Sit down with support\Sit down with support.txt',
    "Stand up": r'D:\HKU\Elderly Assistance\front_following_msliu\PatternRecognition\Stand up with support\LEFT&RIGHT30.txt'
}
sample_dict = dict()
for x in path_dict.keys():
    sample_dict[x] = ""

def LoadSample(index):
    # Read all sample data stored in txt files and convert to lists
    for key in sample_dict.keys():
        with open(path_dict[key]) as f:
            txt_read = f.read().split('\n')  # txt_read is a list with 4500 str items

        mylist = list()
        # convert all str items to list items
        for item in txt_read:
            temp = item.split(',')
            if not index:
                del temp[0]  # delete index
            mylist.append(temp)
        # mylist is a list with 4500 list
        length = len(mylist)
        sample_dict[key] = np.split(np.array(mylist), (length / 150))
    pass

def plotResult(sample):
    print(type(sample), sample.shape)

    time = [i for i in range(150)]

    for i in range(13):
        locals()['sensor%d' % i], = plt.plot(time, sample[:, i], label="sensor %d" % i)
    plt.legend(loc="upper right", )
    y_ticks = np.linspace(0, 30, 16)
    y_index = []
    for i in range(16):
        y_index.append(str(i))
    plt.yticks(y_ticks, y_index)
    plt.ylim(0,50)
    plt.xlabel("Time")
    plt.ylabel("Pressure")
    plt.grid(True)
    plt.show()
    pass

def decisionTree(sample):
    temp_sample = sample
    for i in range(len(sample)):
        temp_sample[i] = temp_sample[i].astype(type(1))
        temp_sample[i].sort()
    pass


if __name__ == '__main__':
    LoadSample(False)
    # decisionTree(sample_dict["Walking with aid_FRONT"])
    plotResult(sample_dict["Walking with aid_RIGHT"][1])
    # print(sample_dict["Walking with aid_RIGHT"], len(sample_dict["Walking with aid_RIGHT"]))