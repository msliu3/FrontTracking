import serial
import numpy as np
import math

import time
from threading import Thread

class Infrared(Thread):

    def __init__(self, frequency = 10):
        #ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=None)
        Thread.__init__(self)
        self.frequency = frequency
        self.raw_data = []
        self.farleft = 0
        self.left = 0
        self.middle = 0
        self.right = 0
        self.farright = 0


    def run(self):
        ser = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=None)
        while 1:
            # print("123124134123515")
            FarLeft = np.zeros(self.frequency)
            FarLeftSum = 0
            Left = np.zeros(self.frequency)
            LeftSum = 0
            Middle = np.zeros(self.frequency)
            MiddleSum = 0
            Right = np.zeros(self.frequency)
            RightSum = 0
            FarRight = np.zeros(self.frequency)
            FarRightSum = 0
            i = 0
            while i < self.frequency:
                try:
                    DATADATA = ser.readline().decode("utf-8")
                    DATA = DATADATA.split()
                    if (len(DATA) == 5):
                        k00 = float(DATA[0])
                        FarLeft[i] = k00
                        FarLeftSum = FarLeftSum + k00
                        k11 = float(DATA[1])
                        Left[i] = k11
                        LeftSum = LeftSum + k11
                        k22 = float(DATA[2])
                        Middle[i] = k22
                        MiddleSum = MiddleSum + k22
                        k33 = float(DATA[3])
                        Right[i] = k33
                        RightSum = RightSum + k33
                        k44 = float(DATA[4])
                        FarRight[i] = k44
                        FarRightSum = FarRightSum + k44
                        i = i + 1



                except BaseException as be:
                    print("error")

            x = FarLeftSum / self.frequency
            y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
            ShowFarLeft = y
            x = LeftSum / self.frequency
            y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
            ShowLeft = y
            x = MiddleSum / self.frequency
            y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
            ShowMiddle = y
            x = RightSum / self.frequency
            y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
            ShowRight = y
            x = FarRightSum / self.frequency
            y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
            ShowFarRight = y
            self.farleft = ShowFarLeft
            self.left = ShowLeft
            self.middle = ShowMiddle
            self.right = ShowRight
            self.farright = ShowFarRight




if __name__ == '__main__':
    If = Infrared()
    If.start()
    # thread1 = Thread(target=If.run, args=())
    # thread1.start()
    while(1):
        print(If.farleft, end="    ")
        print(If.left, end="    ")
        print(If.middle, end="    ")
        print(If.right, end="    ")
        print(If.farright)
        time.sleep(0.3)