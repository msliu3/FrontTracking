from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors
import time

if __name__ == "__main__":
    ss = SoftSkin.SoftSkin()
    ss.build_base_line_data()
    #dataofsoftskin = ss.read_softskin_data()
    while 1:
        ss.serial.flushInput()
        dataofsoftskin = ss.read_softskin_data()

        print(dataofsoftskin)
        time.sleep(1)