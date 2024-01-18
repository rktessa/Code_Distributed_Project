import zmq
import sys 
import numpy as np
import time
import serial
import RPi.GPIO as GPIO
import var_strings_
import logging
import os
import pickle
from wls_class import *
from datetime import datetime

# datetime object containing current date and time
now = datetime.now()

# dd/mm/YYH:M:S
dt_string = now.strftime("%m_%d_%Y__%H_%M_%S")
num_file = sys.argv[1] if len(sys.argv) > 1 else dt_string
ident_vector = [5556,5567,5558,5560,5562]

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


# fare funzione che richiama un codice esterno che usa uwb e wls ricorsivo
# e inizializza le posizioni iniziali delle Mbot, partendo da una sola fissa


# Creo una classe nodo, una per ogni robot nella rete,
# per monitorare la loro attività e risalire alle informazioni 


def main():   
    
    # Code to inizialize 
   
    # Leggere vettore distance una prima volta per inizializzare
    try:
        with open("distances_vector.pkl", 'rb') as file: # mi aspetto file sia un vettore
            dist_vec = pickle.load(file)
            distance_0 = dist_vec[0]
            distance_1 = dist_vec[1]
            distance_2 = dist_vec[2]
            distance_3 = dist_vec[3]
            distance_4 = dist_vec[4]
            z0 = dist_vec
    except FileNotFoundError:
        print("File not found.")
    except EOFError:
        print("No data found in the file.")    
    
        
    #Check what Mbot is runing the code 
    for l in range(len(dist_vec)):
        if dist_vec[l] != 0:
            continue
        else:
            identity = ident_vector[l]  # The Mbot runnning the code is the one that have distanze zero from itself
    print("Sono nel while, qui topic %s" % (topicfilter))
    
    # Process messages from ALL the sockets
    zero_UWB = time.perf_counter()
    
    n0 = 0
    n1 = 0
    n2 = 0
    n3 = 0
    n4 = 0
    

    # Initialize class to compute the position with WLS
    # UNa Mbot sta ferma e funge da ancora e punto di origine del sistema di riferimento che 
    # uso per calcolare la posizione
    # Define Anchors
    An = np.array([[0, 0]])

    # Define sigma_error values in meters
    sigma_error = np.array([np.sqrt(0.1), np.sqrt(0.1), np.sqrt(0.1), np.sqrt(0.1),
                            np.sqrt(0.1)])

    
    n_agents = 5 # 1 Anchor + 4 Mbots moving
    
    x_input = np.array([[10,10],[20,30],[4,5],[30,20]]) # valori casuali per inizializzare e basta
    
    # Initialize the class
    wls_1 = WLSClass_real(An, sigma_error, n_agents, x_input)

    # Initialize the WLS
    
    x_est, P = wls_1.initialization(x_input, z0)

    # create and write the pos for the first time
    with open("position.pkl", 'wb') as file:
                pos_est = [x_est[1], x_est[2]]    
                pickle.dump(pos_est, file)
                print("Prima_pos_est  = ", pos_est)

    last_fraction = time.time() #for print position estimated while running
    while True:
        # Leggere vettore distance
        try:
            with open("distances_vector.pkl", 'rb') as file: # mi aspetto file sia un vettore
                dist_vec = pickle.load(file)
                distance_0 = dist_vec[0]
                distance_1 = dist_vec[1]
                distance_2 = dist_vec[2]
                distance_3 = dist_vec[3]
                distance_4 = dist_vec[4]
                #print("dist_uwb = ",dist_vec)
        except FileNotFoundError:
            print("File not found.")
        except EOFError:
            print("No data found in the file.")    
        
        

        # Leggi messaggi da tutti i PUB, adesso mi interessano solo:
        
        socks = dict(poller.poll()) #poller tracks the availability and number of messages, not the actual messages.
        if subscriber_0 in socks:
            message_0 = subscriber_0.recv_string()
            try:
                port0, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos,y_pos = work(message_0)
                if port0 == "5556":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status0, n0 = Mbot_0.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_0, x_pos, y_pos)
            except Exception as e:
                #print(e)
                pass

        if subscriber_1 in socks:
            message_1 = subscriber_1.recv_string()
            try:
                port1, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos,y_pos = work(message_1)
                if port1 == "5567":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status1, n1 = Mbot_1.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_1, x_pos, y_pos)
            except:
                pass

        if subscriber_2 in socks:
            message_2 = subscriber_2.recv_string()
            try:
                port2, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos, y_pos= work(message_2)
                if port2 == "5558":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status2, n2 = Mbot_2.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_2, x_pos, y_pos)
            except:
                pass


        if subscriber_3 in socks:
            message_3 = subscriber_3.recv_string()
            try:
                port3, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos, y_pos = work(message_3)
                if port3 == "5560":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status3, n3 = Mbot_3.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_3, x_pos, y_pos)
            except:
                pass
        
        if subscriber_4 in socks:
            message_4 = subscriber_4.recv_string()
            try:
                port4, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos, y_pos = work(message_4)
                if port4 == "5562":
                # Argomenti di update: time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance
                    status4, n4 = Mbot_4.update(time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance_4, x_pos, y_pos)
            
            except Exception as e:
                print(e)
                pass
        
       
        # Con le info di posizioni e distanze, posso calcolare con il WLS la posizione mentre si muvono
        
        x0_input = Mbot_0.data[n0-1,12]
        y0_input = Mbot_0.data[n0-1,13]
        #
        x1_input = Mbot_1.data[n1-1,12]
        y1_input = Mbot_1.data[n1-1,13]
        #
        x2_input = Mbot_2.data[n2-1,12]
        y2_input = Mbot_2.data[n2-1,13]
        #
        x3_input = Mbot_3.data[n3-1,12]
        y3_input = Mbot_3.data[n3-1,13]
        #
        x4_input = Mbot_4.data[n4-1,12]
        y4_input = Mbot_4.data[n4-1,13]
        
         # vector x_input necessary for wls
        x_input_est = np.array([[x1_input,y1_input],[x2_input,y2_input],[x3_input,y3_input],[x4_input,y4_input]])
        
        # Step dell'algoritmo wls
        x_est, P = wls_1.WLS2_distributed(x_est, P, x_input_est, dist_vec)
        # salvo nel pickle file x_est
        with open("position.pkl", 'wb') as file:
                pos_est = [x_est[1], x_est[2]]  
                
                pickle.dump(pos_est, file)
        
        if n1 % 20 == 0: #every 20 iteration reinitialize
            
            x_est, P = wls_1.initialization(x_input_est, dist_vec)
        
        
       
        
        
                
        
        dt = time.time() - last_fraction  
        if dt >1:  # print every second
            print("x_input_est = ", x_input_est)
            print("dist_uwb = ",dist_vec)
            print("x_%i = %f \n" %(identity,x_est[1] ))
            print("y_%i = %f \n" %(identity,x_est[2] ))
            last_fraction = time.time()

        if (n0  >= Mbot_0.leng or n1  >= Mbot_1.leng or n2  >= Mbot_2.leng or n3  >= Mbot_3.leng or n4  >= Mbot_4.leng ):  #Prova di scrivere il file e uscire una volta completo
            print("Ho acquisito tutti i campioni") 
            np.savetxt(f, Mbot_1.data, delimiter=' ', fmt='%f') # %d è integer 
            
            np.savetxt(f, Mbot_2.data, delimiter=' ', fmt='%f') # %d è integer
            
            np.savetxt(f, Mbot_3.data, delimiter=' ', fmt='%f') # %d è integer
            
            np.savetxt(f, Mbot_4.data, delimiter=' ', fmt='%f') # %d è integer
            print("Fine Scrittura")
            f.close()
            
            # salvo nel pickle file x_est
            with open("state_number.pkl", 'wb') as file:
                state = 1   
                pickle.dump(state, file)
            

            break

        
        

if __name__ == "__main__":
    
    
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

        
        