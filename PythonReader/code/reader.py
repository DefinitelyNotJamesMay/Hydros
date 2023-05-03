from time import sleep
import serial
import json
import sys

def test_clear():
    for x in range(5):
        print(x)
        sleep(1)

def read_from_usb():
    print("\n"*20)
    with(serial.Serial("COM4", 9600, timeout=1)) as arduino:
        sleep(0.1)
        if arduino.is_open:
            print("Connected")
            while True:
                while arduino.inWaiting()==0: pass
                if  arduino.inWaiting()>0: 
                    answer=arduino.readline()
                    try:
                        data = json.loads(answer)
                        for _ in range(len(data)):
                            sys.stdout.write("\033[F")
                        for key in data:
                            print(key,data[key])
                    except json.JSONDecodeError:
                        pass
                    arduino.flushInput() 
                    sleep(0.2)


if __name__=='__main__':
    read_from_usb()
    # test_clear()