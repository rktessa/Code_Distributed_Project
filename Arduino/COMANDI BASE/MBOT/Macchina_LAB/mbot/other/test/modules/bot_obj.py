class detected_bot:
	def __init__(self, name=None, poseX=None, poseY=None, orientation=None, timestamp=None, trasmitter=5):
		self.poseX       = poseX
		self.poseY       = poseY
		self.name        = name
		self.orientation = orientation
		self.timestamp   = timestamp
		self.trasmitter  = trasmitter


	@staticmethod
	def from_server_string(string): 
		ar = string.split(',')             
		name = None
		poseX = None
		poseY = None
		orientation = None
		tstamp = None
		trasmitter = 5

 		#data coming from cameras are in the form of array data = [c,info_bot1,info_bot2,...] where info_bot1 = (n =.. , x = .. , y = .. , ...)
		for s in ar: 	              
			couple = s.split("=")
			if couple[0] == "n":      
				name=couple[1]
			elif couple[0] == "x":
				poseX=float(couple[1])
			elif couple[0] == "y":
				poseY=float(couple[1])
			elif couple[0] == "t":
				tstamp=float(couple[1])
			elif couple[0] == "a":
				orientation=float(couple[1])
			elif couple[0] == "s":
				trasmitter=int(couple[1])

		return detected_bot(name, poseX, poseY, orientation, tstamp, trasmitter) 


	# create a string with observer estimation data to be sent to cameras client
	def to_server_string(self):
		return "&t=%.3f,x=%.3f,y=%.3f,a=%.3f,s=%,n=" % (self.timestamp, self.poseX, self.poseY, self.orientation,  self.trasmitter) + self.name

class IMU:
	def __init__(self, accX = None, accY=None, acc=None, omega=None, timestamp=None):
		self.accX = accX
		self.accY = accY
		self.acc = acc
		self.omega = omega 
		self.timestamp = timestamp

