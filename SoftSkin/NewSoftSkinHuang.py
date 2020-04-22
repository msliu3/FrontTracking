from SoftSkin import SoftSkin
from DigitalDriver import ControlandOdometryDriver
import time


def ControlBysoftSkin(datanew):
    print(datanew)

if __name__ == "__main__":
    ss = SoftSkin()
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()

    # cd.speed, cd.omega

    while 1:
        data = ss.read_softskin_data()
        try:
            if(data[2]>50):
                if(data[10]>50):
                    if(data[2]-data[10]<100):
                        if(data[2]-data[10]>-100):
                            cd.speed = -0.1
                            cd.omega = 0
                        else:
                            cd.speed = -0.1
                            cd.omega = 0.1
                    else:
                        cd.speed = -0.1
                        cd.omega = 0.1
                else:
                    cd.speed = 0.0
                    cd.omega = 0.0
            else:
                cd.speed = 0.0
                cd.omega = 0.0



        except BaseException as be:
            print("Data error", be)

        ControlBysoftSkin(data)