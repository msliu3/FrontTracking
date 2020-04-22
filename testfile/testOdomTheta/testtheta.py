import threading
import math
from datetime import time
from time import sleep

import GatherData_visualization.gatherIMU as IMU
import DigitalDriver.ControlandOdometryDriver as Control
if __name__ == '__main__':
    imu = IMU.ArduinoRead()
    cd = Control.ControlDriver(record_mode=True)
    cd.start()
    imu.start()
    data_path = "./theta3.txt"
    with open(data_path, 'w') as file:
        while True:
            sleep(0.3)
            # print(-cd.odo.X,-cd.odo.Y)
            print(math.degrees(cd.odo.THETA),imu.imu_robot)
            file.write("%f\t%f\n"%(math.degrees(cd.odo.THETA),imu.imu_robot))