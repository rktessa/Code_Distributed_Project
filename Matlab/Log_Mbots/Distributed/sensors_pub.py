# Here I create a Server PUB that bind an adress, collect the 
# sensors reading and push to multiple clients
# April 2023 Riccardo Tessarin

import zmq
import time
import sys
import random


# Create the port to use, with the adress
# port = "192.168.123.1:5556"
# port = "192.168.123.63:5556"

ip_address = "192.168.123.1"
port = "5556"

if len(sys.argv) > 1:
    port = sys.argv[1]
    str(port)

# Creation of Context, socket definition and address bindings
context =zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://%s:%s" % (ip_address,port)) # %s indicate a string type


# Part in which sensors are read
#
# TO DEVELOP


# Data is published along with a topic. The SUB usually sets
# a filter on these topics oe topic of their interest

while True:
    topic = "10001"
    messagedata = random.randrange(1,215)-80
    print( "%s %s %d" % (topic,port, messagedata)) #Print these 2 data types
    socket.send_string(f"%s %s %d" % (topic,port, messagedata))
    time.sleep(1)