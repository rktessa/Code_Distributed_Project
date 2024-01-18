import zmq
import sys 

# Specify port and IP adress for connections
port = "5556"
port1 = "5567"

ip_address = "192.168.123.1"
ip_address1 = "192.168.123.18"


# socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collectng updates from the first server")
socket.connect("tcp://%s:%s" % (ip_address,port))
print("Mi sono connesso a %s" % port)

#if len(sys.argv) > 2:
socket.connect("tcp://%s:%s" % (ip_address1,port1))
print("Mi sono connesso a %s" % port1)

# Subscribe to topic
topicfilter = sys.argv[1] if len(sys.argv) > 1 else "10001"
socket.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

total_value = 0
print("Sono nel for, qui topic %s" % (topicfilter))

for update_nbr in range(50):
    string = socket.recv_string()
    topic, port, messagedata = string.split()
    total_value = total_value + int(messagedata)
    print(topic,port, messagedata)

    #print("Average messagedata value for topic '%s' was %dF" % (topicfilter, total_value/update_nbr))



