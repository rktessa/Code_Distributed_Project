import socket
import struct
import traceback
import threading
import time
import log_data
from log_data import *

class bcolors:
	HEADER  = '\033[95m'
	OKBLUE  = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL    = '\033[91m'
	ENDC    = '\033[0m'

class Model():

	def __init__(self,PORT):
		self.HOSTNAME = '127.0.0.1'
		self.PORT = PORT
		self.BUFFER_SIZE = 1024
		self.connect = False
		self.client = None			
		self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.server.bind((self.HOSTNAME, self.PORT))
		self.logger = None
		self.parser = None
		self.parser2 = None
		self.t_last = 0
		print('waiting for simulink to start')
		self.server.listen(1)
		self.timer = 0
		self.RESET = 5

	def start(self):
		print('initializing connection...')
		threading.Thread(target=self.__server_thread).start()
		
	def log_output(self,folder):
		self.logger = log_data.controls(folder)
		print(str(self.PORT)+' step 1' )
		
	def __server_thread(self):
		print('Server started on port '+str(self.PORT))
		conn, addr = self.server.accept()
		self.client = conn
		print('Connection adress: ',addr)
		while True:
			try:
				#receive model estimation
				tmp = time.time()
				loop_time = tmp-self.timer
				if loop_time >= self.RESET:
					data = struct.pack('>d',1)
					self.client.send(data)
					print(bcolors.BLUE+'TRIGGER EVENT'+ bcolors.ENDC)
					self.timer = tmp
				print(bcolors.GREEN+'timer count: '+str(loop_time)+ bcolors.ENDC)
			except:
				traceback.print_exc()
				self.client.close()
				break		
		self.server.close()
		
