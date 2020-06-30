from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import time


def ControlBysoftSkin(datanew):
    print(datanew)

if __name__ == "__main__":
    #ss = SoftSkin.SoftSkin()
    #dataofsoftskin = ss.read_softskin_data()
    ser = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=None)
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

        k=np.zeros(80)

        # DATADATA = ser.readline().decode("utf-8")
        #
        # print(DATADATA)

        # sum=0
        # while(j<4):
        #     k = np.zeros(10)
        #     while(i<10):
        #         DATADATA = ser.readline().decode("utf-8")
        #         MM=float(DATADATA)
        #         k[i] = MM
        #         sum = sum + MM
        #         i = i + 1
        #     i=0
        #     sum = sum - max(k) - min(k)
        #     kk[j]=sum/8
        #     j=j+1
        #     sum=0
        # j=0
        # sum=0
        # while(j<4):
        #     sum=sum+kk[j]
        #     j=j+1
        # print(kk[0])

        while(i<80):
            DATADATA = ser.readline().decode("utf-8")
            MM=float(DATADATA)
            k[i]=MM
            sum=sum+MM
            i=i+1
        i=0
        sum=sum-max(k)-min(k)
        k=sum/78
        x=k
        y = 0.00000000000002469574910271750 * x ** 6 - 0.00000000005791417326138540000 * x ** 5 + 0.00000005595190765543280000000 * x ** 4 - 0.00002869659625926520000000000 * x ** 3 + 0.00835326900650030000000000000 * x ** 2 - 1.35378175587794000000000000000 * x + 107.49212259247700000000000000000
        print(y)
        sum=0

        #MMM = MM-0.4
        #Juli = (MMM**(-1/0.912))/0.092903789207
        #print(k)
        #print(Juli)

        #print(DATADATA[0])

