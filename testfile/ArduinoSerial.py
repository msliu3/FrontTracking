import serial
import  time

# from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

if __name__ == "__main__":
    port = "/dev/ttyACM0"

    ser = serial.Serial("COM8", 9600, timeout=None)
    ser.flushInput()

    try:
        while True:
            command = input('Command: ')
            ser.write(bytes(command, encoding='utf-8'))
            print('Command sent! Waiting response...')

            response = ser.readline().decode('utf-8')
            # handle_data = ser.readline().decode("utf-8")
            # handle_data = handle_data.split('\r')[0]
            # handle_data = handle_data.split(',')
            # handle_data = list(map(int, handle_data))
            print('Response received: ', response)
    except:
        print('Failure!')
        ser.close()