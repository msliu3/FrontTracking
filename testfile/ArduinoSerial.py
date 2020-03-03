import serial
import time

# Linux下arduino的端口
port = "/dev/ttyACM0"

ser = serial.Serial('COM5', 9600, timeout=0.05)
ser.flushInput()

for i in range(50):
    ser.readline().decode('utf-8')

try:
    while True:
        # command = input("Command: ")
        # ser.write(bytes(command, encoding='utf-8'))
        # print("Command sent!")

        response = ser.readline().decode('utf-8')
        response = response.split('\r')[0]
        response = response.split(',')
        response = list(map(int, response))
        # print(response, len(response), type(response))
except:
    print('Failure!')
    ser.close()
