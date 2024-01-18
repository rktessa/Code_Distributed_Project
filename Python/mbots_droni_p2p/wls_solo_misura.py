
import sys 
import numpy as np
import time
import serial
import var_strings_
import logging
import os
import pickle
from wls_class import *
from datetime import datetime





def main():   
    
    # Code to inizialize 
  
    

    # Initialize class to compute the position with WLS
    # UNa Mbot sta ferma e funge da ancora e punto di origine del sistema di riferimento che 
    # uso per calcolare la posizione
    # Define Anchors
    An = np.array([[0, 0], [0.0,1.5], [0.95,1.34], [-0.20,1.34]]) #0, 2, 3, 4 

    # Define sigma_error values in meters
    sigma_error = 0.001*np.array([np.sqrt(0.3), np.sqrt(0.3), np.sqrt(0.3), np.sqrt(0.3),
                            np.sqrt(0.3)])

    
    n_agents = 4 # 4 Anchor + 0 Mbots moving
    
    #x_input = np.array([[10,10],[20,30],[4,5],[30,20]]) # valori casuali per inizializzare e basta
    x_input = np.array([])  # Empty if only one Mbot and 4 anchors are used

    # Initialize the class
    wls_1 = WLSClass_real(An, sigma_error, n_agents, x_input)

    # Initialize the WLS
    z0 = np.array([0.92, 0.79, 0.70, 0.91] )
    x_est, P = wls_1.initialization(x_input, z0)
    
    for n in range(100): 

   

        x_input_est = np.array([]) 
        dist_vec = np.array([0.92, 0.79, 0.70, 0.91] )
        
        #[0.5347, 0.001, 0.1642, 0.1126, 0.5112]
        


        # Step dell'algoritmo wls
        x_est, P = wls_1.WLS2_distributed(x_est, P, x_input_est, dist_vec)
       
        print("Soluzione n%i = %f, %f" %(n,x_est[1], x_est[2]))
        
        
                
       
    return x_est
        
        

if __name__ == "__main__":
    
    

    

    try:
        #Initialize the node class
        
        x_est = main()
        sys.exit()
    except KeyboardInterrupt:
        print("Fine")

                   
          
  