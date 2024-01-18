# Here I create a Central control room to start, stop and reset
# all the Mbots from one program
# June 2023 Riccardo Tessarin

import sys
import select
import global_variable_Gateway_
from central_pub_Gateway import *

def input_with_timeout(timeout):
    #print(prompt, end='', flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if ready:
        return sys.stdin.readline().rstrip()
    else:
        return None

# start = 0
# stop = 1
# reset = 2
# calibrate = 3

# Ask the command and deliver to all the MBots
def main():
    while True:
        command = input("Input 'inizio' (o 'fine' per terminare):  ")
        if command == 'fine':
            break  # Esci dal ciclo while esterno

       
        if command == 'inizio':
            global_variable_Gateway_.stato = 2 #Initially set to reset
            while True:
                # Esegui le azioni nel loop interno
                # ...
                #global_variable_.stato = 0
                #pubblish_global_state()
                nuovo_input = input_with_timeout(timeout=0.01)
                
                if nuovo_input is None:
                    #pubblish_global_state()
                   # Nessun input ricevuto entro il timeout, manda comunque lo stato. 
                # Continua l'esecuzione del loop interno
                    pass 
                elif nuovo_input == start:
                    global_variable_Gateway_.stato = 0
                    pubblish_global_state()
                elif nuovo_input == reset:
                    global_variable_Gateway_.stato = 2
                    pubblish_global_state()
                elif nuovo_input == calibrate:
                    global_variable_Gateway_.stato = 3
                    pubblish_global_state()
                elif nuovo_input == 'fine':
                    global_variable_Gateway_.stato = 2 # Manda un reset un ultima volta
                    pubblish_global_state()
                    break
                else:
                    print("The command input does not exist retry")  # If the instruction is write erroneously don't do anything and require again what to do
                    pubblish_global_state()



if __name__ == "__main__":
    
    start = "start"
    stop = "stop"
    reset = "reset"
    calibrate = "calibrate"
  
    topic = "Control"
    try:
        print("Tell me what to do:")
        print("To begin type inizio")
        print("To command Mbots write start, stop, reset or calibrate")
        main()
    except KeyboardInterrupt:
        print("Reset the Mbots")
        global_variable_Gateway_.stato = 2
        pubblish_global_state()