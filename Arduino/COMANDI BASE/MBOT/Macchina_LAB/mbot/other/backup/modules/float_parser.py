import struct 

# parse the data into command + float:
#     data[1]->command
#     data[2:6] -> bytes of the first float to parse
#     data[6:10] -> bytes of the second float (if present)
#     data[10:14] -> bytes of the third float (if present)

# given the message return a touple with command identifier and float value

def parseFloat(data): 
	try:
		if len(data)>=6:
			sum = data[2]
			for i in range(3,6):
				sum = sum + data[i]
			s = list(struct.unpack('4B',sum))
			return (data[1].decode('utf-8'),struct.unpack('<f', struct.pack('4B', *s))[0])
		else:
			return ('o',0)
	except Exception as e:
		print ("float_error: "+str(e))
	
def parseFloat2(data):
	try:
		if len(data)>=10:
			sum = data[6]
			for i in range(7,10):
				sum = sum + data[i]
			s = list(struct.unpack('4B',sum))
			return (data[1].decode('utf-8'),struct.unpack('<f', struct.pack('4B', *s))[0])
		else:
			return ('o',0)
	except Exception as e:
		print ("float_error: "+str(e))
	   
def parseFloat3(data):
	try:
		if len(data)>=14:
			sum = data[10]
			for i in range(11,14):
				sum = sum + data[i]
			s = list(struct.unpack('4B',sum))            
			return (data[1].decode('utf-8'),struct.unpack('<f', struct.pack('4B', *s))[0])
		else:
			return ('o',0)
	except Exception as e:
		print ("float_error: "+str(e))
