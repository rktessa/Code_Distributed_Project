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
		self.DISABLE = False

	def start(self):
		print('initializing connection...')
		threading.Thread(target=self.__server_thread).start()
		
	def log_output(self,folder):
		self.logger = log_data.controls(folder)
		print(str(self.PORT)+' step 1' )
		
	def log_output2(self,folder):
		self.logger = log_data.controls2(folder)
		print(str(self.PORT)+' step 1' )
		
	def __server_thread(self):
		print('Server started on port '+str(self.PORT))
		conn, addr = self.server.accept()
		self.client = conn
		time.sleep(3)
		print('Connection adress: ',addr)
		self.t_last = time.time()
		while True:
			try:
				#receive model estimation
				conv_est = self.client.recv(self.BUFFER_SIZE)
				if conv_est!= b'\x7f\xf8\x00\x00\x00\x00\x00\x00\x7f\xf8\x00\x00\x00\x00\x00\x00':
					x = struct.unpack('>d', struct.pack('8B',*conv_est[0:8]))[0]
					y = struct.unpack('>d', struct.pack('8B',*conv_est[8:16]))[0]
					phi = struct.unpack('>d', struct.pack('8B',*conv_est[16:24]))[0]
					v = struct.unpack('>d', struct.pack('8B',*conv_est[24:32]))[0]
					omega = struct.unpack('>d', struct.pack('8B',*conv_est[32:40]))[0]
					t = time.time()
					self.parser(x,y,phi,v,omega,t)

					'''
					#Estimation Results
					if self.PORT == 28000:
						self.parser(v,omega,angle)
						if self.logger != None:
							#self.logger.output(v,omega)
							t_el = time.time()- self.t_last
							#print('Simulink: '+str(t_el))
							self.logger.trajectory(v,omega,t_el)
							self.t_last = time.time()
					#Feedback controls
					if self.PORT == 27000:
						self.parser(v,omega)
						if self.logger != None:
							#self.logger.output(v,omega)
							t_el = time.time()- self.t_last
							#print('Pyhton: '+str(t_el))
							self.logger.trajectory(v,omega,t_el)
							self.t_last = time.time()
					'''

				else:
					print(bcolors.FAIL+'Refused Data'+bcolors.ENDC)
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
	def set_parser2(self, code):
		self.parser2 = code

	def send_data(self, msg):
		try:
			if not self.DISABLE:
				data = struct.pack('>d',msg)
				self.client.send(data)
				#print(bcolors.OKGREEN+"CORRECTLY SENT"+bcolors.ENDC)
			else:
				print(bcolors.FAIL+"CONNECTION DISABLED"+bcolors.ENDC)
				self.DISABLE = False
		except:
			self.client.close()	
			
	def send_data2(self, msg):
		try:
			data = struct.pack('>d',msg)
			self.client.send(data)
		except:
			self.client.close()				
