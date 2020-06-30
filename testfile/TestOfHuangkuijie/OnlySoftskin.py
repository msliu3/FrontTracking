from SoftSkin import SoftSkin
import time
import serial
import numpy as np
import math

if __name__ == "__main__":
    ss = SoftSkin.SoftSkin()
    ss.build_base_line_data()
    while 1:

        try:
            ss.serial.flushInput()
            dataofsoftskin = ss.read_softskin_data(0)

            print(dataofsoftskin)
            time.sleep(1)
        except BaseException as be:
            print("error")