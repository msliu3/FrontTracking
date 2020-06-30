from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import math
import time

if __name__ == "__main__":

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
        try:
            ss.serial.flushInput()
            dataofsoftskin = ss.read_softskin_data(0)
            print(dataofsoftskin)
            time.sleep(1)

        except  BaseException as be:
            print("error")