import time
import serial

def read_ard(ser):
    ser.flushInput()
    ser.readline().decode('utf-8')
