#! /usr/bin/python3
import zmq
import sys # To play and stop the execution of my code

context_1 = zmq.Context()
#context_2 = zmq.Context()

#  Socket to talk to server, here is the Client part 
print("Connecting to hello world server...")
socket_1 = context_1.socket(zmq.REQ)
socket_1.connect("tcp://192.168.123.1:5555") # this is binding the number 1 server to which I connect

# Try to initialize a second server here
#socket_2 = context_2.socket(zmq.REP)
#socket_2.bind("tcp://192.168.123.1:5555")


def call_1_to_2():
    #  Do 10 REQUEST, waiting each time for a response from server 1
    for request in range(10):
        print(f"Sending request {request} ...")
        socket_1.send_string("Number 1 call number 2")

        #  Get the reply.
        message = socket_1.recv()
        print(f"Received reply from {request} [ {message} ]")
    #sys.exit()

while True: 
    call_1_to_2()


