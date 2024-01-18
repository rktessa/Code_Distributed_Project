# The script waits until a message is receiveid from the main board
# With command start, calibrate and reset the Arduino microcontroller is controlled
#  
# Every Raspberry run a PUB/SUB algorithm
# Jun '23 Riccardo Tessarin


import time
import sys
import serial
import RPi.GPIO as GPIO
import subprocess
import zmq
from pub_to_sub_Mbot import *

port = "5555"
ip_address= "192.168.123.1"

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

'''
def Mbot_data():
    try:
        read_ser = ser.readline().decode("utf-8").rstrip()
        if len(read_ser) > 5: # Per non mandare milioni di righe vuote in giro
            Mbot_send(read_ser)
    except serial.SerialException: #mando una stringa finta intanto per test
            read_ser = 'Data,5562,Inizio,66.16,0.03,-0.12,0.12,-0.00,-0.01,0.01,62.67,96.41,Fine'
            Mbot_send(read_ser)
 '''           
def Mbot_data():
    
    read_ser = ser.readline().decode("utf-8").rstrip()
    if len(read_ser) > 5: # Per non mandare milioni di righe vuote in giro
        Mbot_send(read_ser)
        print(read_ser)
    


if __name__ == '__main__':
    # Setup of the serial communication
    ser=serial.Serial("/dev/ttyUSB0",
                    115200,
                    timeout=0)

    ser.reset_input_buffer()

    if ser.isOpen():
        ser.close()
    ser.open()
    ser.isOpen()
    
    
    # I need to send a first message to say at Arduino that the Serial
    # connection is started. This message is received, but Arduino
    # not register it
    ser.write(b"Hello from Raspberry Pi!\n")
    line = ser.readline().decode('utf-8') #.rstrip()
    #print(line)
    time.sleep(1)
    
    azione_int = 3 # Di default esegue la calibrazione in loop Arduino
    
    while True:
        
    # Attending the command from the Central_pub
    # start = 0
    # stop = 1, dismissed command
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
                ser.write("Start\n".encode())
            elif azione_int == 2 :
                print("Send Reset")
                ser.write("Reset\n".encode())
            elif azione_int == 3 :
                print("Send Calibrate")
                ser.write("Calibrate\n".encode())
            else:
                print("No existing command received, ERROR ERROR!")
        
        except:
            pass

        Mbot_data()
        #line = ser.readline().decode('utf-8').rstrip()
        #print(line)
        
        