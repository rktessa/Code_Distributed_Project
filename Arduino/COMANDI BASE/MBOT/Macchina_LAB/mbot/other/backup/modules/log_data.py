#library to save data on file, for offline debug
import time
import csv
import os
import numpy as np


class log():
	def __init__(self,name):	
		#self.directory = "Log_DATA/%s" % str(time.asctime(time.localtime(time.time())) )
                self.directory = "Log_DATA/%s" % str(name)
                os.makedirs(self.directory)

	def imu(self,IMU):
		self.accX      = IMU.accX
		self.accY      = IMU.accY
		self.acc       = IMU.acc
		self.omega     = IMU.omega
		self.timestamp = IMU.timestamp
		
		file_data = open("%s/%s" % (self.directory,"ImuData.csv"),'a')
		file_data.write("%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.accX,self.accY,self.acc,self.omega,self.timestamp))


	def camera(self,x,y,angolo,timestamp,rec_timestamp):
		self.x = x
		self.y = y
		self.angolo = angolo
		self.timestamp = timestamp
		self.rec_timestamp = rec_timestamp
		file_data3 = open("%s/%s" % (self.directory,"CameraData.csv"),'a') 
		file_data3.write("%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.x,self.y,self.angolo,self.timestamp,self.rec_timestamp)) 
		# rec_timestamp: timestamp at which the frame has been received
		
		
