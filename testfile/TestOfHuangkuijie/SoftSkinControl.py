from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import time


def ControlBysoftSkin(datanew):
    print(datanew)

if __name__ == "__main__":
    ss = SoftSkin.SoftSkin()
    ss.build_base_line_data()
    #dataofsoftskin = ss.read_softskin_data()
    ser = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=None)
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()

    print("1")
    # cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    # cd.start()

    # cd.speed, cd.omega
    #DATADATA = ser.readline().decode("utf-8")
    #DATADATA = DATADATA.split(',')
    i=0
    j=0
    sum=0
    while 1:

        FarLeft=np.zeros(80)
        FarLeftSum=0
        Left=np.zeros(80)
        LeftSum=0
        Middle=np.zeros(80)
        MiddleSum=0
        Right=np.zeros(80)
        RightSum=0
        FarRight=np.zeros(80)
        FarRightSum=0
        i=0
        while i<80:
            try:
                DATADATA = ser.readline().decode("utf-8")
                DATA = DATADATA.split()
                if(len(DATA)==5):
                    k00 = float(DATA[0])
                    FarLeft[i]=k00
                    FarLeftSum=FarLeftSum+k00
                    k11=float(DATA[1])
                    Left[i]=k11
                    LeftSum=LeftSum+k11
                    k22=float(DATA[2])
                    Middle[i]=k22
                    MiddleSum=MiddleSum+k22
                    k33=float(DATA[3])
                    Right[i]=k33
                    RightSum=RightSum+k33
                    k44=float(DATA[4])
                    FarRight[i]=k44
                    FarRightSum=FarRightSum+k44
                    i=i+1
            except BaseException as be:
                print("error")

        x=FarLeftSum/80.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowFarLeft=y
        x = LeftSum / 80.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowLeft=y
        x = MiddleSum / 80.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowMiddle=y
        x = RightSum / 80.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowRight=y
        x = FarRightSum / 80.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowFarRight=y
        print(ShowFarLeft,end='    ')
        print(ShowLeft,end='    ')
        print(ShowMiddle,end='    ')
        print(ShowRight,end='    ')
        print(ShowFarRight)
        ss.serial.flushInput()
        dataofsoftskin = ss.read_softskin_data(0)
        print(dataofsoftskin)
        if(dataofsoftskin[1]>50):
            input()

        print(1)