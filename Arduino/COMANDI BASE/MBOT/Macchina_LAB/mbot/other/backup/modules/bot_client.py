import socket         
import select         
import string          
import sys
import threading       
import time
import configparser


class bcolors:
	HEADER  = '\033[95m'
	OKBLUE  = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL    = '\033[91m'
	ENDC    = '\033[0m'


class BotClient():                  
	def __init__(self, HOSTNAME, PORT, alias):   
		self.alias = alias
		self.parser = None
		self.HOSTNAME = HOSTNAME
		self.PORT = PORT
		self.WAIT_TIME = 5
		self.not_connected_text = 'Unable to connect'
		self.__connect()
            
	def __connect(self):                   
		try :               
			self.connected = False                                        
			self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    
			self.client_socket.settimeout(2)                                          
			self.client_socket.connect((self.HOSTNAME, self.PORT))                    
			self.client_socket.setblocking(False)
			#print(bcolors.OKGREEN+'Connected'+bcolors.ENDC)
			self.client_socket.send((self.alias).encode('utf-8'))
			self.not_connected_text = 'Connection lost'
			self.connected = True
                    
		except Exception as e:                                                
			print (e)
			print (bcolors.FAIL + self.not_connected_text + bcolors.ENDC)
			time.sleep(self.WAIT_TIME)
			self.__connect()
            
	#generate and start a new trhead (flow process) where target is the task to be executed
	def start(self):
		threading.Thread(target=self.__server_thread).start()
        
	#thread target task: receive and save data from the socket channel
	def __server_thread(self):
		print('server started on port ' +str(self.PORT))
		while True:
			read_sockets, write_sockets, error_sockets = select.select([self.client_socket],[],[])			                                                      
			for sock in read_sockets:
				#if socket is a server socket - new connection, accept it
				if sock == self.client_socket:            
					try:
						data = sock.recv(4096).decode('utf-8') 
										               
						if not data:
							print (bcolors.WARNING+"\nDisconnected from server" + bcolors.ENDC)					
							self.__connect()                  
						else:
							#print("parse data:")
							self.parser(data)
					except:                                                 
						continue    

	def send_data(self, msg): 
		if (self.is_connected()):
			try:
				self.client_socket.send(msg.encode('utf-8'))    
				return True
			except:
				self.connected = False
				return False
		else:
			print('not sent...')
			return False
    
	def is_connected(self):                    
		return self.connected	

	def set_parser(self, fun): 
		self.parser=fun
