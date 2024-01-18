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
            z0  = np.array([distance_0,distance_1,distance_2,distance_3,distance_4])
    except FileNotFoundError:
        print("File not found.")
    except EOFError:
        print("No data found in the file.")    
    

    # Initialize class to compute the position with WLS
    # UNa Mbot sta ferma e funge da ancora e punto di origine del sistema di riferimento che 
    # uso per calcolare la posizione
    # Define Anchors
    An = np.array([[0, 0], [0,3.9], [-1.1,6.0], [0.0,6.0]]) #0, 2, 3, 4 

    # Define sigma_error values in meters
    sigma_error = np.array([np.sqrt(0.3), np.sqrt(0.3), np.sqrt(0.3), np.sqrt(0.3),
                            np.sqrt(0.3)])

    
    n_agents = 4 # 4 Anchor + 0 Mbots moving
    
    #x_input = np.array([[10,10],[20,30],[4,5],[30,20]]) # valori casuali per inizializzare e basta
    x_input = np.array([])  # Empty if only one Mbot and 4 anchors are used

    # Initialize the class
    wls_1 = WLSClass_real(An, sigma_error, n_agents, x_input)

    # Initialize the WLS
    z0 = wls_1.z_sensor(x_input, z0)  #! questo fondamentale
    x_est, P = wls_1.initialization(x_input, z0)

    # create and write the pos for the first time
    with open("position.pkl", 'wb') as file:
                pos_est = [x_est[1], x_est[2]]    
                pickle.dump(pos_est, file)
                print("Prima_pos_est  = ", pos_est)

    last_fraction = time.time() #for print position estimated while running


    n_mis = 0
    distances = np.zeros((10000,6))
    x_estimated = np.zeros((10000,2))
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
                dist_vec = np.array([distance_0,distance_2,distance_3,distance_4])
                distances[n_mis,:-2] = dist_vec
                
                #print("dist_uwb = ",dist_vec)
        except FileNotFoundError:
            print("File not found.")
        except EOFError:
            print("No data found in the file.")    
        
        n_mis = n_mis + 1 

        x_input_est = np.array([]) 
        # Step dell'algoritmo wls

        x_est, P = wls_1.WLS2_distributed(x_est, P, x_input_est, dist_vec)
        # salvo nel pickle file x_est
        with open("position.pkl", 'wb') as file:
                pos_est = [x_est[1], x_est[2]]  
                x_estimated[n_mis,:] = pos_est
                pickle.dump(pos_est, file)
        
       # if n1 % 20 == 0: #every 20 iteration reinitialize
            
       #     x_est, P = wls_1.initialization(x_input_est, dist_vec)
        
   
        dt = time.time() - last_fraction  
        if dt >1:  # print every second
            #print("x_input_est = ", x_input_est)
            print("dist_uwb = ",dist_vec)
            print("x = %f " %(x_est[1] ))
            print("y = %f " %(x_est[2] ))
            last_fraction = time.time()
           
        if n_mis > 10000-1:
            print("Ho acquisito tutti i campioni")  
            
            np.savetxt(f, distances, delimiter=' ', fmt='%f') # %d è integer
            np.savetxt(f, x_estimated, delimiter=' ', fmt='%f') # %d è integer      
            print("Fine Scrittura")
            f.close()




            break
    return distances
        
        

if __name__ == "__main__":
    
    
    file = "distanze_misurate_"+ num_file+ ".txt"

    f = open(file, 'a') 
    
    

    try:
        #Initialize the node class
        
        distances = main()
        sys.exit()
    except KeyboardInterrupt:
        np.savetxt(f, distances, delimiter=' ', fmt='%f') # %d è integer      
        
        print("Fine scrittura")  

                   
          
  