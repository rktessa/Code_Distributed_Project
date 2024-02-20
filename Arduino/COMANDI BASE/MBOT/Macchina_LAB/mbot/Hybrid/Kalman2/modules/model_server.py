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
		self.BUFFER_SIZE = 4096
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
				S_x = [0,0,0,0]
				S_p = [0,0,0]
				P_x = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
				P_p = [0,0,0,0,0,0,0,0,0]
				
				if conv_est!= b'\x7f\xf8\x00\x00\x00\x00\x00\x00\x7f\xf8\x00\x00\x00\x00\x00\x00':
					S_x[0] = struct.unpack('>d', struct.pack('8B',*conv_est[0:8]))[0]
					S_x[1] = struct.unpack('>d', struct.pack('8B',*conv_est[8:16]))[0]
					S_x[2] = struct.unpack('>d', struct.pack('8B',*conv_est[16:24]))[0]
					S_x[3] = struct.unpack('>d', struct.pack('8B',*conv_est[24:32]))[0]
					
					S_p[0] = struct.unpack('>d', struct.pack('8B',*conv_est[32:40]))[0]
					S_p[1] = struct.unpack('>d', struct.pack('8B',*conv_est[40:48]))[0]
					S_p[2] = struct.unpack('>d', struct.pack('8B',*conv_est[48:56]))[0]
					
					v = struct.unpack('>d', struct.pack('8B',*conv_est[56:64]))[0]
					omega = struct.unpack('>d', struct.pack('8B',*conv_est[64:72]))[0]
					
					P_x[0] = struct.unpack('>d', struct.pack('8B',*conv_est[72:80]))[0]
					P_x[1] = struct.unpack('>d', struct.pack('8B',*conv_est[80:88]))[0]
					P_x[2] = struct.unpack('>d', struct.pack('8B',*conv_est[88:96]))[0]
					P_x[3] = struct.unpack('>d', struct.pack('8B',*conv_est[96:104]))[0]
					P_x[4] = struct.unpack('>d', struct.pack('8B',*conv_est[104:112]))[0]
					P_x[5] = struct.unpack('>d', struct.pack('8B',*conv_est[112:120]))[0]
					P_x[6] = struct.unpack('>d', struct.pack('8B',*conv_est[120:128]))[0]
					P_x[7] = struct.unpack('>d', struct.pack('8B',*conv_est[128:136]))[0]
					P_x[8] = struct.unpack('>d', struct.pack('8B',*conv_est[136:144]))[0]
					P_x[9] = struct.unpack('>d', struct.pack('8B',*conv_est[144:152]))[0]
					P_x[10] = struct.unpack('>d', struct.pack('8B',*conv_est[152:160]))[0]
					P_x[11] = struct.unpack('>d', struct.pack('8B',*conv_est[160:168]))[0]
					P_x[12] = struct.unpack('>d', struct.pack('8B',*conv_est[168:176]))[0]
					P_x[13] = struct.unpack('>d', struct.pack('8B',*conv_est[176:184]))[0]
					P_x[14] = struct.unpack('>d', struct.pack('8B',*conv_est[184:192]))[0]
					P_x[15] = struct.unpack('>d', struct.pack('8B',*conv_est[192:200]))[0]
					
					P_p[0] = struct.unpack('>d', struct.pack('8B',*conv_est[200:208]))[0]
					P_p[1] = struct.unpack('>d', struct.pack('8B',*conv_est[208:216]))[0]
					P_p[2] = struct.unpack('>d', struct.pack('8B',*conv_est[216:224]))[0]
					P_p[3] = struct.unpack('>d', struct.pack('8B',*conv_est[224:232]))[0]
					P_p[4] = struct.unpack('>d', struct.pack('8B',*conv_est[232:240]))[0]
					P_p[5] = struct.unpack('>d', struct.pack('8B',*conv_est[240:248]))[0]
					P_p[6] = struct.unpack('>d', struct.pack('8B',*conv_est[248:256]))[0]
					P_p[7] = struct.unpack('>d', struct.pack('8B',*conv_est[256:264]))[0]
					P_p[8] = struct.unpack('>d', struct.pack('8B',*conv_est[264:272]))[0]
					
					print('x: '+str(S_x[0]))
					print('y: ' + str(S_x[1]))
					print('phi:'+str(S_p[0]))
					'''
					print('v:'+str(v))
					print('omega:'+str(omega))
					print('p1:' + str(P_p[0]))
					print('p2:' + str(P_p[1]))
					print('p3:' + str(P_p[2]))
					print('p4:' + str(P_p[3]))
					print('p5:' + str(P_p[4]))
					'''
					#print('p:'+str(p[0]))
					t = time.time()
					self.parser2(S_x,S_p,v,omega,t)
					self.parser(P_x,P_p,t)

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
