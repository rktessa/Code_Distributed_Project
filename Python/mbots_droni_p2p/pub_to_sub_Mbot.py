import zmq 
import time
import serial
import var_strings_
import pickle

#identity a seconda del numero di Mbot devi selezionare il num corretto.
num = 1
# pub_to_sub_Mbot.py
port = var_strings_.port_list[num] 
ip_address = var_strings_.ip_list[num]

topic = "Data"

context =zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://%s:%s" % (ip_address,port)) # %s indicate a string type


'''
def Mbot_send(stringa):
    # read from pickle file
    try:
        with open("position.pkl", 'rb') as file: # mi aspetto file sia un vettore
            positions = pickle.load(file)
            #print(positions)
            stringa = stringa + ',' + str(positions[0])+ ',' + str(positions[1])
            print(stringa)
    except FileNotFoundError:
        print("File not found.")
        pass
    except EOFError:
        print("No data found in the file.")  
        


    # Here I create a Server PUB that bind an adress
    # ! Every MBOT bind a different port and address (its own ip) !
    # stringa is data send by "initial_script_3_Mbot.py"
    # Inizio,29.84,-0.23,-0.02,-0.16,-0.07,-0.02,-0.02,110.87,106.05,Fine,x_pos,y_pos
    
    socket.send_string("%s,%s,%s" %(topic,port,stringa))
'''    
   

def Mbot_send(stringa):
    try:
        inizio, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, fine = stringa.split(",")
        try:
            with open("position.pkl", 'rb') as file: # mi aspetto file sia un vettore
                positions = pickle.load(file)
                #print(positions)
                stringa = stringa + ',' + str(positions[0])+ ',' + str(positions[1])
                #print(stringa)
                # Here I create a Server PUB that bind an adress
                # ! Every MBOT bind a different port and address (its own ip) !
                # stringa is data send by "initial_script_3_Mbot.py"
                # Inizio,29.84,-0.23,-0.02,-0.16,-0.07,-0.02,-0.02,110.87,106.05,Fine,x_pos,y_pos

                socket.send_string("%s,%s,%s" %(topic,port,stringa))
        except FileNotFoundError:
            print("File not found.")
            
        except EOFError:
            print("No data found in the file.") 
        
    
    except Exception as e:
        print(e) # Se non è la stringa corretta non la leggo
        #print("Ho ricevuto qualcosa, ma non lo uso ")
        #print(message) # In questo modo visualizzo i messaggi inviati da Arduino e monitoro il suo funzionamento
        pass
