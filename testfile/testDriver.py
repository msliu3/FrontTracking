import DigitalDriver.ControlandOdometryDriver as CD
import threading
import time

if __name__ == '__main__':
    cd = CD.ControlDriver()

    thread_control_driver = threading.Thread(target=cd.control_part, args=())
    thread_control_driver.start()

    cd.speed = 0.1
    time.sleep(2)
    # cd.speed = 0