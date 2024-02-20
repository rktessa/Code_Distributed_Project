#library to save data on file, for offline debug
import time
import csv
import os
import numpy as np


class measures():
	def __init__(self,name):	
		#self.directory = "Log_DATA/%s" % str(time.asctime(time.localtime(time.time())) )
                self.directory = "Log_measures/%s" % str(name)
                os.makedirs(self.directory)

	def imu(self,accX = None, accY=None, acc=None, omega=None, timestamp=None, trig = 0):
		self.accX      = accX
		self.accY      = accY
		self.acc       = acc
		self.omega     = omega
		self.timestamp = timestamp
		self.trig      = trig
		
		file_data = open("%s/%s" % (self.directory,"ImuData.csv"),'a')
		file_data.write("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.accX,self.accY,self.acc,self.omega,self.timestamp,self.trig))


	def camera(self,x,y,angolo,delay,rec_timestamp):
		self.x = x
		self.y = y
		self.angolo = angolo
		self.delay = delay
		self.rec_timestamp = rec_timestamp
		file_data3 = open("%s/%s" % (self.directory,"CameraData.csv"),'a') 
		file_data3.write("%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.x,self.y,self.angolo,self.delay,self.rec_timestamp)) 
		# rec_timestamp: timestamp at which the frame has been received
		
class output():		
	def __init__(self,name):	
		#self.directory = "Log_DATA/%s" % str(time.asctime(time.localtime(time.time())) )
                self.directory = "Log_measures/%s" % str(name)
                
	def camera(self,x,y,angolo,delay,rec_timestamp):
		self.x = x
		self.y = y
		self.angolo = angolo
		self.delay = delay
		self.rec_timestamp = rec_timestamp
		file_data3 = open("%s/%s" % (self.directory,"CameraData.csv"),'a') 
		file_data3.write("%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.x,self.y,self.angolo,self.delay,self.rec_timestamp)) 
		# rec_timestamp: timestamp at which the frame has been received

	def trajectory(self,x,y,phi,t):
		self.x = x
		self.y = y
		self.phi = phi
		self.t = t
		file_data4 = open("%s/%s" % (self.directory,"Trajectory.csv"),'a') 
		file_data4.write("%.9f,%.9f,%.9f,%.9f\n" % (self.x,self.y,self.phi,self.t)) 
		
	def control(self,s,r,v_r,omega_r,t_r):
		self.s1 = s[0]
		self.s2 = s[1]
		self.s3 = s[2]
		self.s4 = s[3]
		self.r1 = r[0]
		self.r2 = r[1]
		self.r3 = r[2]
		self.v_r = v_r
		self.omega_r = omega_r
		self.t_r = t_r
		file_data2 = open("%s/%s" % (self.directory,"Controls.csv"),'a') 
		file_data2.write("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.s1,self.s2,self.s3,self.s4,self.r1,self.r2,self.r3,self.v_r,self.omega_r,self.t_r)) 
		# rec_timestamp: timestamp at which the frame has been received
		
	def kalman(self,p,q,t):
		self.p1 = p[0]
		self.p2 = p[1]
		self.p3 = p[2]
		self.p4 = p[3]
		self.p5 = p[4]
		self.p6 = p[5]
		self.p7 = p[6]
		self.p8 = p[7]
		self.p9 = p[8]
		self.p10 = p[9]
		self.p11 = p[10]
		self.p12 = p[11]
		self.p13 = p[12]
		self.p14 = p[13]
		self.p15 = p[14]
		self.p16 = p[15]
		self.q1 = q[0]
		self.q2 = q[1]
		self.q3 = q[2]
		self.q4 = q[3]
		self.q5 = q[4]
		self.q6 = q[5]
		self.q7 = q[6]
		self.q8 = q[7]
		self.q9 = q[8]
		self.t = t	
		file_data2 = open("%s/%s" % (self.directory,"Kalman.csv"),'a') 
		file_data2.write("%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % (self.p1,self.p2,self.p3,self.p4,self.p5,self.p6,self.p7,self.p8,self.p9,self.p10,self.p11,self.p12,self.p13,self.p14,self.p15,self.p16,self.q1,self.q2,self.q3,self.q4,self.q5,self.q6,self.q7,self.q8,self.q9,self.t)) 
		# rec_timestamp: timestamp at which the frame has been received
		
class controls():
	def __init__(self,name):	
		#self.directory = "Log_DATA/%s" % str(time.asctime(time.localtime(time.time())) )
                self.directory = "Log_control/%s" % str(name)
                os.makedirs(self.directory)

	def output(self,v,omega,t):
		self.v = v
		self.omega = omega
		self.t = t
		file_data2 = open("%s/%s" % (self.directory,"Controls.csv"),'a') 
		file_data2.write("%.9f,%.9f,%.9f\n" % (self.v,self.omega,self.t)) 
		# rec_timestamp: timestamp at which the frame has been received
		
	def trajectory(self,x,y,t):
		self.x = x
		self.y = y
		self.t = t
		file_data4 = open("%s/%s" % (self.directory,"Trajectory.csv"),'a') 
		file_data4.write("%.9f,%.9f,%.9f\n" % (self.x,self.y,self.t)) 
		
class controls2():
	def __init__(self,name):	
		#self.directory = "Log_DATA/%s" % str(time.asctime(time.localtime(time.time())) )
                self.directory = "Log_control2/%s" % str(name)
                os.makedirs(self.directory)

	def output(self,v,omega,t):
		self.v = v
		self.omega = omega
		self.t = t
		file_data2 = open("%s/%s" % (self.directory,"Controls2.csv"),'a') 
		file_data2.write("%.9f,%.9f,%.9f\n" % (self.v,self.omega,self.t)) 
		# rec_timestamp: timestamp at which the frame has been received
		
	def trajectory(self,x,y,t):
		self.x = x
		self.y = y
		self.t = t
		file_data4 = open("%s/%s" % (self.directory,"Trajectory2.csv"),'a') 
		file_data4.write("%.9f,%.9f,%.9f\n" % (self.x,self.y,self.t)) 
