import os
import sys
import time
import numpy as np
pwd = os.path.abspath(os.path.abspath(__file__))
father_path = os.path.abspath(os.path.dirname(pwd) + os.path.sep + "..")
sys.path.append(father_path)



import SoftSkin.SoftSkin as SS
import threading
import multiprocessing


def loop(event):
    while True:
        time.sleep(1)
        end = input("end?")
        if end == "end":
            print("------------------------------------------------")
            event.set()

def loop2(ss):

    while True:
        ss.read_softskin_data(0)

if __name__ == '__main__':

    ss = SS.SoftSkin()
    ss.build_base_line_data()
    event = threading.Event()
    event.clear()
    p1 = threading.Thread(target=loop, args=(event,))
    p1.start()
    p2 = threading.Thread(target=loop2, args=(ss,))
    p2.start()
    data_path = father_path + os.path.sep + "resource" + os.path.sep + "softskin.txt"
    print(data_path)

    with open(data_path, 'w') as file:
        while True:
            time.sleep(0.2)
            # print("%f"%cd.odo.Radius)

            k = str(np.array(ss.raw_data) - np.array(ss.base_data))
            # print(k,"\n")
            k = k.strip("[")
            k = k.replace("]","\n")
            k = k.replace(",", " ")
            k = k.replace("  ", " ")
            k = k.replace("  ", " ")
            k = k.replace("  ", " ")
            k = k.replace("  ", " ")
            k = k.replace("  ", " ")
            
            k = k.strip(" ")
            k = k.replace(" ", "\t")
            print(k,"\n")
            file.write(k)
            # file.write("sasasasasasa \n")
            if event.is_set():
                break
        file.close()
        # thread_control_driver.join()
        # thread_start.join()
        # p1.join()