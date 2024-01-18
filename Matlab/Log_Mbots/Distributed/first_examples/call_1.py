#! /usr/bin/python3
# Code for distributed meassaging code in python and ZeroMQ
# April 2023
# Riccardo Tessarin
import time
import zmq

# Call 1, gira sul primo Raspebbery
# Ha un server e un client dentro,
# in due context separati

# Server Part
context = zmq.Context() 

socket_1 = context.socket(zmq.REP)
socket_1.bind("tcp://192.168.123.1:5555")

# Try to initialize a second part as a client here
print("Connecting to number 2 server...")

socket_2 = context.socket(zmq.REQ)
socket_2.bind("tcp://192.168.123.63:5559")

def call_2_from_1():
    #  Do 10 REQUEST, waiting each time for a response from server 1
    for request in range(3):
        print(f"Sending request {request} ...")
        socket_2.send_string("Number 1 Client call number 2 Server")

        #  Get the reply.
        message = socket_2.recv()
        print(f"Received reply from {request} [ {message} ]")


while True:
    #  Wait for next request from client 2
    message_1 = socket_1.recv()
    print(f"Received request: {message_1}")

    #  Do some 'work'
    print("Node 1 is working")
    time.sleep(1)

    #  Send reply back to client
    socket_1.send_string("I am Server 1, ready to work")

    # Try to call the client number 2
    call_2_from_1()
