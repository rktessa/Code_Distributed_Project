import struct
import sys,time
import signal
import traceback
from time import ctime,sleep
import glob,struct
import threading
from multiprocessing import Process,Manager,Array
import serial

class bcolors:
	HEADER  = '\033[95m'
	OKBLUE  = '\033[94m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL    = '\033[91m'
	ENDC    = '\033[0m'

#--------------------------arduino object CLASS---------------------------------------------------------------------
class arduino_bot():
	def __init__(self):
		print ("Initialization mBot...")
		signal.signal(signal.SIGINT, self.exit)        
		self.manager = Manager()
		self.__selectors = self.manager.dict()
		self.buffer = [] #store all the incoming msg that must be parsed
		self.isParseStart = False
		self.exiting = False
		self.parser = None
		self.is_float = False
		self.name = None                       
        
	def set_parser(self, parser):
		self.parser=parser
       
	def excepthook(self, exctype, value, traceback):
		self.close()	
	
	#initilize the connection with the specified serial port 
	def startWithSerial(self, port):
		self.device = mSerial() #-->JUMP ON THE OTHER CLASS
		self.device.start(port)
		self.start()           
            
	def start(self):
		sys.excepthook = self.excepthook
		print('start serial comunication...')
		th = threading.Thread(target=self.__onRead,args=(self.onParse,))
		th.start()

    #function that recursivly check if the serial port is ready and check the stack of incoming msg to parse them
    #iterativly check the buffer and verify which kind of data the parser has to analize, then call parser
	def __onRead(self,callback):
		while True:
			if(self.exiting==True):
				break
			try:	
				if self.device.isOpen()==True:
					#get the number of bytes in the receive buffer (incoming msg)
					n = self.device.inWaiting()
					for i in range(n):
						r = self.device.read()
						if r==b'&' and len(self.buffer)==0:
							self.is_float = True #float data to be parsed
						if not r==b'\n':
							self.buffer+=[r]
						if r==b'\r' and not(len(self.buffer)==0):
							callback(self.buffer)
							#reset buffer and float fleg
							self.buffer=[]
							self.is_float = False
						sleep(0.001)

			except Exception as ex:
				print (bcolors.FAIL+"__onRead error: "+str(ex)+ bcolors.ENDC)
				traceback.print_exc()
				self.close()
				sleep(1)
             
	def onParse(self, s):
		if not (self.parser == None):
			self.parser(self,s)
        
	def close(self):
		self.device.close()

	def exit(self, signal, frame):
		self.exiting = True
		sys.exit(0)
    
	# send a string
	def writeString(self, s):  
		s = "%"+s+"$"                                                 
		self.__writePackage(bytearray([ord(c) for c in s])) 

	#send to the bot command+value
	def sendCommand(self,cname,value):
		try:
			#print('step 1')
			#print(("%" + cname).encode('utf-8') + struct.pack('f',value) +("$").encode('utf-8'))
			self.__writePackage(("%" + cname).encode('utf-8') + struct.pack('f',value) +("$").encode('utf-8'))
		except Exception as e:
			print (bcolors.WARNING+str(e)+ bcolors.ENDC)

	#send a package of data to bot    
	def __writePackage(self,pack):
		#print('step 2')
		self.device.writePackage(pack)


#----------------------------------mSERIAL PORT CLASS-------------------------------------------------------------------
class mSerial():                     
	ser = None
    
	#def __init__(self):
	#	print (self)

	#generate serial connection to the specified channel (port,baudrate)
	def start(self, port):
		self.ser = serial.Serial(port = port,baudrate = 9600)  
    
	def device(self):
		return self.ser 

	def serialPorts(self): 
		if sys.platform.startswith('win'):                                           #This string contains a platform identifier (windows) 
			ports = ['COM%s' % (i + 1) for i in range(256)]
		elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):  #(linux)
			ports = glob.glob('/dev/tty[A-Za-z]*')
		elif sys.platform.startswith('darwin'):                                      #(Mac os)
			ports = glob.glob('/dev/tty.*')
		else:
			raise EnvironmentError('Unsupported platform')
		result = []
		#generate serial connectio obj channel
		for port in ports:
			s = serial.Serial()   
			s.port = port
			s.close()
			result.append(port)   
		return result


	#send data to the serial port
	def writePackage(self,package):
		#print('step 3')
		self.ser.write(package)       
		sleep(0.01)

	#read data from the serial port
	def read(self):
		return self.ser.read()        

	#Get the state of the serial port(whether it’s open=>free to communicate)
	def isOpen(self): 
		return self.ser.isOpen()      

	#Get the number of bytes in the input buffer
	def inWaiting(self): 
		return self.ser.inWaiting()   

	def close(self):
		self.ser.close()
