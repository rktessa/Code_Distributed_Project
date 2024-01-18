# DEMO TO TEST STUFF
# The script waits for when a message is receiveid from the main board
# With command start, calibrate and reset the Arduino microcontroller is controlled
#  
# Every Raspberry run a PUB/SUB algorithm
# Jun '23 Riccardo Tessarin


import time
import subprocess
import zmq
import sys

port = "5555"
ip_address= "192.168.123.1"
ip_address= "localhost"

# socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.RCVTIMEO, 0) #If there is no messages raise an Error

# Connessione di tutti i nodi al programma centrale di comando
print("Collecting updates from the Central server")
socket.connect("tcp://%s:%s" % (ip_address,port))
print("Mi sono connesso  a %s:%s" %(ip_address, port))

# Subscribe to topic
topicfilter = sys.argv[1] if len(sys.argv) > 1 else "Control"
socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

def Mbot_data():
    # Here I create a Server PUB that bind an adress, collect the 
    # sensors reading and push to multiple clients
    # ! Every MBOT bind a different port and address (its own ip) !
   
    ip_address_local = "*"
    port = "5556"
    topic = "Data"
    context =zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://%s:%s" % (ip_address_local,port)) # %s indicate a string type
    
    # Reading data from Arduino
    # Inizio,29.84,-0.23,-0.02,-0.16,-0.07,-0.02,-0.02,110.87,106.05,Fine
    #read_ser = ser.readline().decode("utf-8").rstrip()
    read_ser = "Inizio,29.84,-0.23,-0.02,-0.16,-0.07,-0.02,-0.02,110.87,106.05,Fine"  #esempio di stringa
    socket.send_string("%s,%s,%s" %(topic,port,read_ser))
    

if __name__ == '__main__':

        
    
    azione_int = 3 # Di default esegue la calibrazione in loop Arduino
    
    while True:
        
    # Attending the command from the Central_pub
    # start = 0
    # stop = 1
    # reset = 2
    # calibrate = 3
        
        # Se non ci sono nuovi messaggi non aspetta e va avanti
        try:
            string = socket.recv_string() # Ricezione comando da Command Central
            topic, azione = string.split(" ")
            azione_int = int(azione)
            print("Comando ricevuto: ", azione_int)
        
            if azione_int == 0 :
                print("Send Start:")
               
            elif azione_int == 2 :
                print("Send Reset")
                
            elif azione_int == 3 :
                print("Send Calibrate")
               
            else:
                print("No existing command received, ERROR ERROR!")
        
        except:
            pass

        Mbot_data()
        #line = ser.readline().decode('utf-8').rstrip()
        #print(line)
        
        