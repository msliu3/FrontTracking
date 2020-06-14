from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import math
import time

if __name__ == "__main__":
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
    ser = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=None)
    i=0
    j=0
    sum=0
    base=0
    balance=0
    start=0
    ss = SoftSkin.SoftSkin()
    ss.build_base_line_data()
    while 1:
        #print("keepworking")
        try:
            ss.serial.flushInput()
            dataofsoftskin = ss.read_softskin_data(0)
            if(len(dataofsoftskin)==13):
                if(dataofsoftskin[12]>10):
                    cd.omega=-0.15
                    print("turn right")

                else:
                    cd.omega=0


        except  BaseException as be:
            print("error")

        print("X=%.3fm,  Y=%.3fm,  THETA=%.2f" % (cd.position[0], cd.position[1], cd.position[2] / math.pi * 180))
        time.sleep(0.5)
