import serial
import  time

if __name__ == "__main__":
    port = "/dev/ttyACM0"

    ser = serial.Serial(port, 9600, timeout=1)
    ser.flushInput()

    try:
        while True:
            # command = input('Command: ')
            # ser.write(bytes(command, encoding='utf-8'))
            # print('Command sent!')
            # time.sleep(2)
            response = ser.readline().decode('utf-8')
            print('Response received: ',response)
    except:
        print('Failure!')
        ser.close()