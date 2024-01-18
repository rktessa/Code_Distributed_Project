import zmq
import sys 
import numpy as np
import time
import serial
import RPi.GPIO as GPIO
import var_strings_
import logging
import os

num_file = sys.argv[1] if len(sys.argv) > 1 else "10"

# Specify port and IP address for connections
port_0 = var_strings_.port_0
port_1 = var_strings_.port_1
port_2 = var_strings_.port_2
port_3 = var_strings_.port_3
port_4 = var_strings_.port_4


ip_address_local = var_strings_.ip_address_local
ip_address_0 = var_strings_.ip_address_0
ip_address_1 = var_strings_.ip_address_1
ip_address_2 = var_strings_.ip_address_2
ip_address_3 = var_strings_.ip_address_3
ip_address_4 = var_strings_.ip_address_4


# socket to talk to server
context = zmq.Context()

topicfilter = "Data"

# Socket per le prove in locale
# socket = context.socket(zmq.SUB)
# print("Collecting updates from the first Mbot")
# socket.connect("tcp://%s:%s" % (ip_address_local,port))
# topicfilter = sys.argv[1] if len(sys.argv) > 1 else "Data"
# socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter)
# print("Mi sono connesso a %s" % port)


# Sockets degli MBOTs
# Zero
subscriber_0 = context.socket(zmq.SUB)
subscriber_0.connect("tcp://%s:%s" % (ip_address_0,port_0))
subscriber_0.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

# Uno
subscriber_1 = context.socket(zmq.SUB)
subscriber_1.connect("tcp://%s:%s" % (ip_address_1,port_1))
subscriber_1.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

# Due
subscriber_2 = context.socket(zmq.SUB)
subscriber_2.connect("tcp://%s:%s" % (ip_address_2,port_2))
subscriber_2.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

# Tre
subscriber_3 = context.socket(zmq.SUB)
subscriber_3.connect("tcp://%s:%s" % (ip_address_3,port_3))
subscriber_3.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

# Quattro
subscriber_4 = context.socket(zmq.SUB)
subscriber_4.connect("tcp://%s:%s" % (ip_address_4,port_4))
subscriber_4.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

# Initialize poll set to handle multiple message socket
poller = zmq.Poller()
poller.register(subscriber_0, zmq.POLLIN)
poller.register(subscriber_1, zmq.POLLIN)
poller.register(subscriber_2, zmq.POLLIN)
poller.register(subscriber_3, zmq.POLLIN)
poller.register(subscriber_4, zmq.POLLIN)

# Creo una classe nodo, una per ogni robot nella rete,
#  per monitorare la loro attività e risalire alle informazioni ? 

class Node:
    def __init__(self, port):
        self.leng = 1000
        self.port = port # the identity of my node, I use TCP for convenience
        self.time = 0 # Time from the last measure
        self.last_time = 0 # To check if the node is alive
        self.data = np.zeros((self.leng, 12)) #Inizialize a matrix to fill with all the datas from the mbots
        self.n = 0 # To register the n times the data is written

        self.r = 6.4/2 # radius of the wheel in cm
        self.L = 11.2  # interasse in cm
    def update(self, time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance):
        
        # self.data[self.n,0] = float(time_mess)
        # self.time = float(time_mess)
        self.data[self.n,0] = time.perf_counter()
        
       
        self.data[self.n,1] = float(acc_x)
        self.data[self.n,2] = float(acc_y)
        self.data[self.n,3] = float(acc_z)

        self.data[self.n,4] = float(gyro_x)
        self.data[self.n,5] = float(gyro_y)
        self.data[self.n,6] = float(gyro_z)

        omega_l = float(rpm_l)*(np.pi *2) /60
        omega_r = float(rpm_r)*(np.pi *2) /60

        vel_linear = self.r/2 *(omega_l + omega_r) # cm/s
        omega_encoder = (self.r/self.L)*(omega_l - omega_r) # rad/s

        self.data[self.n,7] = vel_linear
        self.data[self.n,8] = omega_encoder

        self.data[self.n,9]  = distance
        self.data[self.n,10]  = rpm_l
        self.data[self.n,11]  = rpm_r
        print(self.data[self.n,:])
        self.n = self.n + 1
        status = 1
        self.last_time = time.perf_counter()
        
        return status, self.n
    
    def check_is_alive(self):
        if (time.perf_counter() - self.last_time) >= 10: # se è più di 10 secondi che non riceve info lo dichiaro morto
            print("Node with port number %s is dead" % self.port)
            # Qui capire poi come escluderre da MDS usando quel nodo finchè non torna in vita?
            stop = 0
            return stop

# To read UWB data 
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


def work(message):
    # topic, port, time_mess, acc_x, acc_y, acc_z,  ==> Cosa mi aspetto di ricevere
    #cosa ricevo
    # b'Data,5562,Inizio,66.16,0.03,-0.12,0.12,-0.00,-0.01,0.01,62.67,96.41,Fine'
        try:
            topic, port,inizio, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, fine = message.split(",") 
            print(message)
            return port, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r
        except Exception as e:
            print(e) # Se non è la stringa corretta non la leggo
            #print("Ho ricevuto qualcosa, ma non lo uso ")
            print(message) # In questo modo visualizzo i messaggi inviati da Arduino e monitoro il suo funzionamento
            pass
        # If you want to use for math convert into float, these strings objects!    
        

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
    

    task_0 = True
    print("Sono nel while, qui topic %s" % (topicfilter))
    
    # Process messages from ALL the sockets
    zero_UWB = time.perf_counter()
    
    n0 = 0
    n1 = 0
    n2 = 0
    n3 = 0
    n4 = 0
    while True:

        # Acquisizione UWB, finchè non ho almeno una lettura di tutte le distanze
        # non inizio, dura circa tre secondi, il tempo di essere certi che tutti 
        # ricevano almeno una delle misure
        while (time.perf_counter()- zero_UWB < 3.0):
            try:
                tx, rx, identity_now, distance  = lettura_uwb()
                print(tx, rx, identity_now, distance)
                if identity_now == zero:
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
            


        # Leggi messaggi da tutti i PUB
        socks = dict(poller.poll())
        if subscriber_0 in socks:
            message_0 = subscriber_0.recv_string()
            try:
                port0, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r = work(message_0)
                if port0 == "5556":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status0, n0 = Mbot_0.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_0)
            except Exception as e:
                print(e)
                pass

        if subscriber_1 in socks:
            message_1 = subscriber_1.recv_string()
            try:
                port1, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r = work(message_1)
                if port1 == "5567":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status1, n1 = Mbot_1.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_1)
            except:
                pass

        if subscriber_2 in socks:
            message_2 = subscriber_2.recv_string()
            try:
                port2, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r = work(message_2)
                if port2 == "5558":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status2, n2 = Mbot_2.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_2)
            except:
                pass


        if subscriber_3 in socks:
            message_3 = subscriber_3.recv_string()
            try:
                port3, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r = work(message_3)
                if port3 == "5560":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status3, n3 = Mbot_3.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_3)
            except:
                pass
        
        if subscriber_4 in socks:
            message_4 = subscriber_4.recv_string()
            try:
                port4, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r = work(message_4)
                if port4 == "5562":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status4, n4 = Mbot_4.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_4)
            
            except Exception as e:
                print(e)
                pass
        
        # Lettura a ogni ciclo del UWB e update delle varie distanze
        try:
            tx, rx, identity_now, distance  = lettura_uwb()
            if identity_now == zero:
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

        if (n0  >= Mbot_0.leng or n1  >= Mbot_1.leng or n2  >= Mbot_2.leng or n3  >= Mbot_3.leng or n4  >= Mbot_4.leng ):  #Prova di scrivere il file e uscire una volta completo
            print("Ho acquisito tutti i campioni") 
            np.savetxt(f, Mbot_1.data, delimiter=' ', fmt='%f') # %d è integer 
            #f.write("\n")
            np.savetxt(f, Mbot_2.data, delimiter=' ', fmt='%f') # %d è integer
            #f.write("\n")
            np.savetxt(f, Mbot_3.data, delimiter=' ', fmt='%f') # %d è integer
            #f.write("\n")
            np.savetxt(f, Mbot_4.data, delimiter=' ', fmt='%f') # %d è integer
            print("Fine Scrittura")
            f.close()
            break

        
        

if __name__ == "__main__":
    
    # SET UWB MESSAGE SERIAL READING
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)
    file = "data_Mbot_"+ num_file+ ".txt"

    f = open(file, 'a') 
    
    

    try:
        #Initialize the node class
        Mbot_0 = Node("5556")
        Mbot_1 = Node("5567")
        Mbot_2 = Node("5558")
        Mbot_3 = Node("5560")
        Mbot_4 = Node("5562")
        main()
        sys.exit()
    except KeyboardInterrupt:
        np.savetxt(f, Mbot_1.data, delimiter=' ', fmt='%f') # %d è integer 
            #f.write("\n")
        np.savetxt(f, Mbot_2.data, delimiter=' ', fmt='%f') # %d è integer
            #f.write("\n")
        np.savetxt(f, Mbot_3.data, delimiter=' ', fmt='%f') # %d è integer
            #f.write("\n")
        np.savetxt(f, Mbot_4.data, delimiter=' ', fmt='%f') # %d è integer
        print("Fine scrittura")  

                   
          
        
        # Quando abbastanza campioni sono stati acquisiti scrivi nel file di log e termina esecuzione
            
                # for i in range(len(Mbot_0.data)):
                #     f.write(str(Mbot_0.data[i,:]))
                #     f.write('\n')
                # for i in range(len(Mbot_1.data)):
                #     f.write(str(Mbot_1.data[i,:]))
                #     f.write('\n')
                # for i in range(len(Mbot_2.data)):
                #     f.write(str(Mbot_2.data[i,:]))
                #     f.write('\n')
                # for i in range(len(Mbot_3.data)):
                #     f.write(str(Mbot_3.data[i,:]))
                #     f.write('\n')
                
   

            # After updates check if the node are still here, working
            #Mbot_0.check_is_alive()
            #Mbot_1.check_is_alive()

        
        