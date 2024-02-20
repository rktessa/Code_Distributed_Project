import socket
import struct
import traceback
import threading
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
		print('waiting for simulink to start')
		self.server.listen(1)

	def start(self):
		print('initializing connection...')
		threading.Thread(target=self.__server_thread).start()
		
	def log_output(self,folder):
		self.logger = log_data.controls(folder)
		
	def __server_thread(self):
		print('Server started on port '+str(self.PORT))
		conn, addr = self.server.accept()
		self.client = conn
		print('Connection adress: ',addr)

		while True:
			try:
				#receive model estimation
				conv_est = self.client.recv(self.BUFFER_SIZE)
				if conv_est!= b'\x7f\xf8\x00\x00\x00\x00\x00\x00\x7f\xf8\x00\x00\x00\x00\x00\x00':
					v = struct.unpack('>d', struct.pack('8B',*conv_est[0:8]))[0]
					omega = struct.unpack('>d', struct.pack('8B',*conv_est[8:16]))[0]
					#print('velocity: '+str(v))
					#print('omega: ' + str(omega))
					if self.logger != None:
						self.logger.output(v,omega)
				else:
					#print(bcolors.FAIL+'Refused Data'+bcolors.ENDC)
					if self.logger != None:
						self.logger.output(0,0)
				if not len(conv_est):
					self.client.close()
					continue
			except:
				traceback.print_exc()
				self.client.close()
				break		

		self.server.close()
		
	def set_parser(self, code):
		self.parser = code

	def send_data(self, msg):
		try:
			data = struct.pack('>d',msg)
			self.client.send(data)
		except:
			self.client.close()	
