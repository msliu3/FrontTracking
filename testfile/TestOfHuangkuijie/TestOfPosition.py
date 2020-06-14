from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import numpy as np
import math

if __name__ == "__main__":

    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
