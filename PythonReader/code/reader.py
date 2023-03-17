from time import sleep
import serial, sqlite3

def create_db():
    con = sqlite3.connect("hydros.db")
    cur = con.cursor()

class HydrosDb:

    def __init__(self, name):
        self.name = name
        self.con = sqlite3.connect("hydros.db")
        self.cur = self.con.cursor()

    def make_tables_if_not_exists():
        self.cur.execute

def read_from_usb():
    with(serial.Serial("COM4", 9600, timeout=1)) as arduino:
        sleep(0.1)
        if arduino.is_open:
            print("Connected")
            while True:
                while arduino.inWaiting()==0: pass
                if  arduino.inWaiting()>0: 
                    answer=arduino.readline()
                    print(answer)
                    arduino.flushInput() 


if __name__=='__main__':
    read_from_usb()