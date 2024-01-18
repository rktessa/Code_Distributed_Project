import zmq
import sys 
import numpy as np
import time
# Specify port and IP adress for connections
port = "5556"
port2 = "5567"

ip_address_local = "localhost"
ip_address_1 = "192.168.123.95"

#Identity index, questa cosa serve da settare diversa
#  in ogni  sub node in base a quale macchina gira per 
# far si ch sappia chi è lui nel programma, è fondamentale
#  per salvare in modo corretto la posizione relativa agli altri
# Legenda: 5556 = 0, 5567 = 1 e poi verrano gli altri
identity_index = 1

# socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting updates from the first Mbot")
socket.connect("tcp://%s:%s" % (ip_address_local,port))
print("Mi sono connesso a %s" % port)

'''
print("Collecting updates from the second Mbot")
socket.connect("tcp://%s:%s" % (ip_address,port2))
print("Mi sono connesso a %s" % port2)
'''
# Subscribe to topic
topicfilter = sys.argv[1] if len(sys.argv) > 1 else "Data"
socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter)


# Creo una classe nodo, una per ogni robot nella rete,
#  per monitorare la loro attività e risalire alle informazioni ? 

class Node:
    def __init__(self, port):
        self.port = port # the identity of my node, I use TCP for convenience
        self.time = 0 # Time from the last measure
        self.last_time = 0 # To check if the node is alive
        self.data = np.zeros((2000, 10)) #Inizialize a matrix to fill with all the datas from the mbots
        self.n = 0 # To register the n times the data is written

        self.r = 6.4/2 # radius of the wheel in cm
        self.L = 11.2  # interasse in cm
    def update(self, time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance):
        
        self.data[self.n,0] = float(time_mess)
        self.time = float(time_mess)
       
        self.data[self.n,1] = float(acc_x)
        self.data[self.n,2] = float(acc_y)
        self.data[self.n,3] = float(acc_z)

        self.data[self.n,4] = float(gyro_x)
        self.data[self.n,5] = float(gyro_y)
        self.data[self.n,6] = float(gyro_z)


        vel_linear = self.r/2 *(float(rpm_l) + float(rpm_r))
        omega_encoder = (self.r/self.L)*(float(rpm_l) - float(rpm_r))

        self.data[self.n,7] = vel_linear
        self.data[self.n,8] = omega_encoder

        self.data[self.n,8]  = distance
        
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



if __name__ == "__main__":
    
    f = open('data_Mbot_00.txt', 'w')
    '''# Code to inizialize UWB
    mesg = {}
    
    # SET OUTPUT MESSAGE
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)
    '''

    #Initialize the node class
    n1 = Node("5556")
    n2 = Node("5567")
 

    print("Sono nel while, qui topic %s" % (topicfilter))
    zero = time.perf_counter()
    while True:
        string = socket.recv_string()
        # topic, port, time_mess, acc_x, acc_y, acc_z,  ==> Cosa mi aspetto di ricevere
        try:
            topic, port,inizio, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, fine = string.split(",") 
        except: # Se non è la stringa corretta non la leggo
            pass
        # If you want to use for math convert into float, these strings objects!    
        
        if (time.perf_counter() - zero) >= 1.0 :
            print(topic, port, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r)
            zero = time.perf_counter()

        '''# Read the UWB
        mesg = rl.readline().decode("utf-8")
        dist_vector = mesg.split(" ")
        
        if (len(dist_vector)!=6):
          mesg = rl.readline().decode("utf-8")
          dist_vector = mesg.split(" ")
        '''  

        # Create different locations for data coming from differen nodes
        if port == "5556":
            # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
            status1, n = n1.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, 1.0)
            
            if n1  == 20:  #Prova di scrivere il file e uscire una volta completo
                print("Ho acquisito tutti i campioni")
                for i in range(len(n1.data)):
                    f.write(n1.data[i,:])
                    f.write('\n')
                sys.exit("Ending of the aquisition")

        elif port == "5567" :
             
           status2 = n2.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, 1.0)  

        # After updates check if the node are still here, working
        n1.check_is_alive()
        #n2.check_is_alive()

        
        
