import time
import os
import traceback
import platform
import atexit
import configparser
import urllib
import traceback
import math as ma

import sys
sys.path.insert(0,'modules/')
from multi_bot  import *
from bot_client import *
import log_data 
from bot_obj import *
from float_parser import *

#-----code parameters-----------
COM_TIME_MAX = 30
TIME_NAME_MAX = 20 		#max time it will wait until resending the alias request
TIME_P_MAX = 15 		#max comunication time before resending ping request for sync
ERROR_COUNTER_LIMIT = 20 	#trial to resend data after error code
LOG_DATA = False
SHOW_DATA = False
READY = False
ready_c = 0
ard_com_time = 0
tmp_time = 0
error_counter = 0
error_counter1 = 0
good_data = 0
name_dir = 0
tloop = time.time()


# parser for data coming from arduino through serial-----------------------------------------------------------------
def bot_parser(bot_l, data):
        global bot_name, ard_com_time, READY, ready_c, tmp_time, error_counter, error_counter1, good_data, client, logger, LOG_DATA,SHOW_DATA
        try:
		#---(&)---> command with float data (gyro)--------
                if data[0] == b'&':
                        good_data +=1
                        tup = parseFloat(data) #get acc (in g = 9.81m/s2)

                        if data[1] == b'd':			
                                tup2=parseFloat2(data) #get ang velocity (in deg/s)
                                tup3=parseFloat3(data)
                                imu_data = data_conversion(tup[1],tup2[1],tup3[1])
                                acc = ma.sqrt((imu_data[0])**2 + (imu_data[1])**2)
                                data  = IMU(imu_data[0],imu_data[1], acc, imu_data[2], time.time()-ard_com_time)
                                if LOG_DATA:
                                        logger.imu(data)
                                if SHOW_DATA:
                                        print("accx: "+str(imu_data[0]))
                                        print("accy: "+str(imu_data[1]))
                                        print('acc: '+str(acc))
                                        print('omega: '+str(imu_data[2]))
                                        print('-------------------------------')


                        elif data[1] == b'h': #ask or stop the gyro T request (gyro period of acquisition)
                                print (bcolors.OKGREEN+"Gyro T: "+ str(tup[1])+bcolors.ENDC)
                                
                        elif data[1] == b't':
                                print(bcolors.OKGREEN+'Acquisition interval: '+ str(tup[1]) +bcolors.ENDC)
		        
		#----normal verbose command-----------------------
                elif data[0] == b'n': 	#send the name
                        if bot_name == None :
                                for i in range(0,len(data)):
                                        data[i]=data[i].decode('utf-8')
                                        print(data[i])
                                bot_name = ''.join(data[1:-1])
                        print('name: '+ bot_name)
		      
                elif data[0] == b'p': 	#ping response for sync time
                        ard_com_time += time.time()-tmp_time
                        if ready_c >= COM_TIME_MAX:
                                READY = True
                        else:
                                ready_c += 1
                                tmp_time = time.time()
                                bot_l.writeString('p')
				            
                elif data[0] == b'o':   #errors flag
                        print (bcolors.WARNING+"Error code: "+data[1].decode('utf-8')+bcolors.ENDC)
                        if var_dict.has_key(data[1].decode('utf-8')):
                                time.sleep(0.005)
                                if error_counter < ERROR_COUNTER_LIMIT:
                                        error_counter +=1
                                        bot_l.writeCommand(data[1].decode('utf-8'), var_dict[data[1].decode('utf-8')])
			   
                else:	#case of unespected data
                        print(bcolors.WARNING+'strange data:...'+bcolors.ENDC)
                        print(data)
                        for i in range(0,len(data)):
                                data[i]=(struct.unpack('B',data[i]))
                        print (data)
                        error_counter1 +=1
                        
        except:
                traceback.print_exc()


#-----parser for data coming from server terminal or server cameras---------------------------------------------------
def client_parser(data): 
	global bot, is_point_to_point, ask_go, is_stop_bot, error_counter1, good_data, client, bot_name, name_dir, logger, LOG_DATA, SHOW_DATA
	try:
		#command parser --> $ means that comunication is from the server terminal (user)
		if data =='enable,1':
			print(bcolors.OKGREEN+'Connected to server'+bcolors.ENDC)

		if data[0] == '$':
			if data[1] != "u":
				print (bcolors.OKBLUE+"-->Server to bot command: "+data[1:]+bcolors.ENDC)

			if data[1] == 'd':   #send velocitiees (abs,rel)	
				val = float(data[3:])
				bot.sendCommand(data[1:3],val)
				
			elif data[1] == 'v' or data[1] == 't':
                                val = 0
                                bot.sendCommand(data[1],val)

			elif data[1] == 'c': #recalibrate acc          
				bot.writeString('c')

			elif data[1] == 'e': #exit program                     
				exit_handler()
				sys.exit()

			elif data[1] == 'l': #start gyro acquisition                      
				bot.writeString('l')

			elif data[1] == 'n': #get bot name    
				print ('ask name :...')
				if bot_name == None:
					bot.writeString('n')
					print("bot not recognized: ask again...")
				elif client != None:
					client.send_data('n'+bot_name)

			elif data[1] == 'g': #set color (testing)
				print ('reset colors :...')
				bot.writeString('g')
				
			elif data[1] == 'h': #set color (testing)
				bot.writeString('h')

			elif data[1] == 'p': #get data statistics   
				if client != None: 
					client.send_data('p1'+str(error_counter1))
					client.send_data('p2'+str(good_data))

			elif data[1:] == 'run': #start bot run
				print('start motion...')
				bot.writeString('r')

			elif data[1:] == 'rest': #stop bot
				print('stop motion...')
				bot.writeString('s')
				
			elif data[1:] == 'stop':
                                print('stop acquisition...')
                                bot.writeString('m')
                                LOG_DATA = False
                                SHOW_DATA = False
                                
			elif data[1:] == 'start':
                                bot.writeString('c')
                                print('start acquisition...')
                                logger = log_data.log(name_dir)
                                name_dir = name_dir+1
                                LOG_DATA = True
                                SHOW_DATA = True
                                
                                
		        
	except:
		traceback.print_exc()
		


def exit_handler(): 
	global bot, client
	print ("Exit handler:")
	try:
                bot.device.ser.close()
                bot.close()
	except:
		print ("Bot not connected")
	print ("bot system closed")
	if 'file_acc' in globals():
		file_acc.close()



#---------------MAIN LOOP---------------------------------------------------------------------------------------------------

if __name__== "__main__":
	try:
		while True:
			client = None
			#-----bot configuration setting----------------------------------------
			conf = configparser.ConfigParser()   		  
			conf.read(os.path.join(os.path.abspath(os.path.dirname(__file__)), '', 'settings/bot_client.ini'))  
			alias = conf.get("set1","Alias")                                                
			   
			#-----initialize serial connection with the arduino--------------------
			bot = arduino_bot()
			try:
				if platform.system()=='Linux':              
					bot.startWithSerial('/dev/ttyUSB0') 
				elif platform.system()=='Windows':
					bot.startWithSerial('COM4')
			except:
				print (bcolors.FAIL+"Serial device not connected" + bcolors.ENDC)   
			time.sleep(1)
			
			#arduino data parser
			bot.set_parser(bot_parser)
			check_time = time.time()

			#-----serial comunication time evaluation-----------------------------
			print ("Obtain comunication time...")
			tmp_time = time.time();
			bot.writeString('p')  #ping arduino
			check_time = time.time()

			while not READY:                               
				if time.time()-check_time>TIME_P_MAX:      
					print (bcolors.WARNING+"Restart sync"+ bcolors.ENDC)
					READY = False
					ready_c = 0
					ard_com_time = 0
					tmp_time = time.time();
					bot.writeString('p')  
					check_time = time.time()  
			READY = False
			ready_c = 0
			ard_com_time = ard_com_time/(2*COM_TIME_MAX) #evaluate time of comm as average 
			print (bcolors.OKGREEN+"Comunication time: "+str(ard_com_time)+ bcolors.ENDC)

			tmp = [0,0]
			dt = time.time() - tloop
			print(bcolors.OKGREEN+"Loop time: "+str(dt)+ bcolors.ENDC)
			tloop = time.time()   
			READY = False    
		
	except SystemExit as e:
		print (bcolors.OKBLUE+"User exit"+ bcolors.ENDC)
		exit_handler()
	except:
		traceback.print_exc()

