from DigitalDriver import ControlandOdometryDriver
import time

if __name__ == "__main__":
    cd = ControlandOdometryDriver.ControlDriver(left_right=1)
    cd.start()
    cd.omega=-0.2
    time.sleep(1)
    cd.omega=0
