from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import math

if __name__ == "__main__":

    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
    ss = SoftSkin.SoftSkin()
    ss.build_base_line_data()
    #dataofsoftskin = ss.read_softskin_data()
    ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=None)
    print("HAVE START SENSOR")
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
    print("HAVE START CD")
    #print("HHHHH")

    # cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    # cd.start()

    # cd.speed, cd.omega
    #DATADATA = ser.readline().decode("utf-8")
    #DATADATA = DATADATA.split(',')
    i=0
    j=0
    sum=0
    base=0
    balance=0
    start=0
    while 1:

        FarLeft=np.zeros(40)
        FarLeftSum=0
        Left=np.zeros(40)
        LeftSum=0
        Middle=np.zeros(40)
        MiddleSum=0
        Right=np.zeros(40)
        RightSum=0
        FarRight=np.zeros(40)
        FarRightSum=0
        i=0
        while i<40:
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

        x=FarLeftSum/40.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowFarLeft=y
        x = LeftSum / 40.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowLeft=y
        x = MiddleSum / 40.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowMiddle=y
        x = RightSum / 40.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowRight=y
        x = FarRightSum / 40.0
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        ShowFarRight=y
        # print(ShowFarLeft,end='    ')
        # print(ShowLeft,end='    ')
        # print(ShowMiddle,end='    ')
        # print(ShowRight,end='    ')
        # print(ShowFarRight)

        try:
            LeftRightBalance = ShowFarLeft+ShowLeft-ShowRight-ShowFarLeft
            FrontBackBalance = (ShowRight+ShowLeft)/2
            ss.serial.flushInput()
            dataofsoftskin = ss.read_softskin_data(0)
            k = 1
            #print(dataofsoftskin)
            if(start!=1):
                print(ShowFarLeft, end='    ')
                print(ShowLeft, end='    ')
                print(ShowMiddle, end='    ')
                print(ShowRight, end='    ')
                print(ShowFarRight)
                print("LeftRightBalance=",end="")
                print(LeftRightBalance,end=" ")
                print("FrontBackBalance=",end=" ")
                print(FrontBackBalance)
            if (dataofsoftskin[1] > 30):
                base = LeftRightBalance
                balance = FrontBackBalance
                print("start")
            if (dataofsoftskin[0] > 30):
                start = 1
                print("begin")
            if ((FrontBackBalance > balance + 4) & (cd.omega == 0)&(start==1)):
                print("go back")
                cd.speed=-0.1
            if ((FrontBackBalance < balance - 3) & (cd.omega == 0)&(start==1)):
                print("go ahead")
                cd.speed=0.1
            if (dataofsoftskin[12] > 30):
                cd.speed = 0
                cd.omega = 0
                print("stop")
                start = 0
            if ((FrontBackBalance > balance - 2) & (FrontBackBalance < balance + 2)):
                cd.speed = 0
                #print("stop")
            if(dataofsoftskin[11] > 30):
                input()



        except BaseException as be:
            print("error")

        print("X=%.3fm,  Y=%.3fm,  THETA=%.2f" % (cd.position[0], cd.position[1], cd.position[2] / math.pi * 180))