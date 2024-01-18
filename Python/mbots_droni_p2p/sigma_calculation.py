import pickle
import random
import time
import serial
import sys
import numpy as np

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
    mesg = mesg.replace("\r\n", "")
    tx, rx, identity, distance = mesg.split(" ") # Divido la stringa e ottengo gli argomenti
    
    return int(tx), int(rx), int(identity), float(distance)



def main():   
    
    # Code to inizialize UWB
    mesg = {}
    zero = 0
    uno = 1
    due = 2
    tre = 3
    quattro = 4

    distance_0 = 0 # 5556
    distance_1 = 0 # 5567
    distance_2 = 0 # 5558
    distance_3 = 0 # 5560
    distance_4 = 0 # 5562

    
    distance_array = np.zeros([1000,5])

        
        
    i = 0
    while True:
        try:
            with open("state_number.pkl", 'rb') as file:
                state = pickle.load(file)
            
        except FileNotFoundError:
            print("File not found.")
        except EOFError:
            print("No data found in the file.")

        # Control if the state variable is active (program is running)
        if state == 0:
            try:
                tx, rx, identity_now, distance  = lettura_uwb()
                #print(tx, rx, identity_now, distance)
                if identity_now == zero:
                    i = i +1
                    #print("dentro if zero")
                    distance_0 = distance
                elif identity_now == uno:
                    #print("dentro if uno")
                    distance_1 = distance
                elif identity_now == due:
                    #print("dentro if due")
                    distance_2 = distance  
                elif identity_now == tre:
                    #print("dentro if tre")
                    distance_3 = distance 
                elif identity_now == quattro:
                    #print("dentro if quattro")
                    distance_4 = distance 
                else:
                    print("Wrong identity, do nothing")
                
                
            except:
                print("Error during read or decode") 
                pass
            distance_vec = [distance_0, distance_1, distance_2, distance_3, distance_4]
            print(distance_vec)
                
            print(i)
            distance_array[i-1,:] = distance_vec
            if i == 1000: # quando hai cento campioni validi esci
                #print(distance_array)
                break            

            # write the resulting distance vector using pickle
            # Save the distance vector to a file using pickle
            
        else:
            break

    return distance_array

if __name__ == "__main__":
    
    # SET UWB MESSAGE SERIAL READING
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)
    
    try:
        distance_array = main()
        variance_vector = np.var(distance_array[1:,:],0)
        print(variance_vector)
        # creo file state.pkl
        with open("state_number.pkl", 'wb') as file: #reinizializzo a 0 lo state
            state = 0   
            pickle.dump(state, file)
            
        # creo file sigma_error.pkl
        with open("sigma_error.pkl", 'wb') as file: #reinizializzo a 0 lo state
            sigma_error = variance_vector   
            pickle.dump(sigma_error, file)
            
        sys.exit()
        print("End of UWB Work")
    except KeyboardInterrupt:
        print("End of UWB Work")