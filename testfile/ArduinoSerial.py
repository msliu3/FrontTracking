import serial
from SoftSkin import SoftSkin
import time
import numpy as np

if __name__ == "__main__":
    ss = SoftSkin.SoftSkin()

    try:
        while True:
            command = input('Command: ')
            ss.serial.write(bytes(command, encoding='utf-8'))
            print('Command sent!')
            ss.read_softskin_data()
            # handle_data = ser.readline().decode("utf-8")
            # handle_data = handle_data.split('\r')[0]
            # handle_data = handle_data.split(',')
            # handle_data = list(map(int, handle_data))

    except:
        print('Failure!')
        ss.serial.close()