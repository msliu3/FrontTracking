from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import serial
import time


def ControlBysoftSkin(datanew):
    print(datanew)

if __name__ == "__main__":
    #ss = SoftSkin.SoftSkin()
    #dataofsoftskin = ss.read_softskin_data()
    ser = serial.Serial(port="/dev/ttyACM1", baudrate=9600, timeout=None)
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()

    # cd.speed, cd.omega
    #DATADATA = ser.readline().decode("utf-8")
    #DATADATA = DATADATA.split(',')


    while 1:
        DATADATA = ser.readline().decode("utf-8")
        print(DATADATA)

        #print(DATADATA[0])


