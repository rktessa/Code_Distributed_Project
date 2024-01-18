import zmq
import sys 
import numpy as np
import time
# Specify port and IP adress for connections
port = "5556"
port2 = "5567"

ip_address_local = "localhost"
ip_address_1 = "192.168.123.26" #Refered

#Identity index, questa cosa serve da settare diversa
#  in ogni  sub node in base a quale macchina gira per 
# far si ch sappia chi è lui nel programma, è fondamentale
#  per salvare in modo corretto la posizione relativa agli altri
# Legenda: 5556 = 0, 5567 = 1 e poi verrano gli altri
identity_index = 1

# socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting updates from the first server")
socket.connect("tcp://%s:%s" % (ip_address_1,port))
print("Mi sono connesso a %s" % port)

'''
print("Collectng updates from the second server")
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
        self.acceleration = np.zeros(3) #vettore accelerazioni
        self.distance = 0 #vettore distanza da ogni altro nodo
        self.gyro = np.zeros(3)
    

    def update(self, time_mess, acc_x, acc_y, acc_z, distance):
        self.acceleration[0] = float(acc_x)
        self.acceleration[1] = float(acc_y)
        self.acceleration[2] = float(acc_z)
        self.distance = distance
        self.time = float(time_mess)
        status = 1
        return status
    
    def check_is_alive(self):
        if (time.perf_counter() - self.time) >= 10: # se è più di 10 secondi che non riceve info lo dichiaro morto
            print("Node with port number %s is dead" % self.port)
            # Qui capire poi come escluderre da MDS usando quel nodo finchè non torna in vita?
            stop = 0
            return stop


if __name__ == "__main__":

    n1 = Node("5556")
    n2 = Node("5567")
 
    print("Sono nel for, qui topic %s" % (topicfilter))
    zero = time.perf_counter()
    while True:
        string = socket.recv_string()
        topic, port, time_mess, accX, accY, accZ, speX, speY, speZ, rotX, rotY, rotZ = string.split(" ") 
        # If you want to use for math convert into float, these strings objects!    
        
        if (time.perf_counter()-zero >= 1.0) :
            print(topic,port, time_mess, accX, accY, accZ, speX, speY, speZ, rotX, rotY, rotZ)
            zero = time.perf_counter()

        # Create different locations for data coming from differen nodes
        '''if port == "5556":
            dist_vector = [dist_0, dist_1, dist_2, dist_3, dist_4, dist_5]
            status1 = n1.update(time_mess, acc_x, acc_y, acc_z, dist_vector[identity_index])

        elif port == "5567" :
           dist_vector = [dist_0, dist_1, dist_2, dist_3, dist_4, dist_5]  
           status2 = n2.update(time_mess, acc_x, acc_y, acc_z, dist_vector[identity_index])  

        # After updates check if the node are still here, working
        n1.check_is_alive()
        n2.check_is_alive()
        '''