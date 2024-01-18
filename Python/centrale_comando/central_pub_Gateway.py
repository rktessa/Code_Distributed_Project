import zmq
import var_strings_Gateway
# Code that send to all the mbots the messages


port = var_strings_Gateway.port
ip_address = var_strings_Gateway.ip_address
ip_address_local = var_strings_Gateway.ip_address_local


# Creation of Context, socket definition and address bindings
context =zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://%s:%s" % (ip_address_local,port)) # %s indicate a string type



def pubblish_global_state():
    from global_variable_Gateway_ import stato
    # Parte della funzione che si occupa di mandare il messaggio, quando viene invocato nel main
    zip_filter = "Control"
    stato_str = str(stato)
    socket.send_string(f"%s %s"% (zip_filter, stato_str))
    #print(stato_str) # To test PRINT THIS STRING
    #context.term()



if __name__ == "__main__":
    zip_filter = "Control"
    while True:
        try:
            pubblish_global_state()
            print("Mandato")
        except KeyboardInterrupt:
            stato_stop = "1"
            socket.send_string(f"%s %s"% (zip_filter, stato_stop)) #send a last stop to all the nodes and then die
            socket.close()    
