
import random
import time
import serial
import sys


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

def lettura_uwb():
    mesg = rl.readline().decode("utf-8")
    print(mesg)
    
    


def main(): 
    while True:
        try:
            lettura_uwb()    
        except Exception as e:
            print(e)
            
            pass

if __name__ == "__main__":
    
    # SET UWB MESSAGE SERIAL READING
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)

    try:
        main()
        
        print("End of UWB Work")
        sys.exit()
    except KeyboardInterrupt:
        print("End of UWB Work")