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
        DATADATA = ser.readline().decode("utf-8")
        print(DATADATA)

    