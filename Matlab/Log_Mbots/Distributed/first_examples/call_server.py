#! /usr/bin/python3
# Code for distributed meassaging code in python and ZeroMQ
# April 2023
# Riccardo Tessarin
import time
import zmq


# Try to write a simple request reply function


context_1 = zmq.Context()
socket_1 = context_1.socket(zmq.REP)
socket_1.bind("tcp://192.168.123.1:5555")

# Try to initialize a second part as a client here
#context_2 = zmq.Context()
#socket_2 = context_2.socket(zmq.REQ)
#socket_2.bind("tcp://192.168.123.1:5556")


while True:
    #  Wait for next request from client
    message = socket_1.recv()
    print(f"Received request: {message}")

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    socket_1.send_string("World")