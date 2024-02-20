import traceback
import time
import sys
sys.path.insert(0,'modules/')
import model_server 

if __name__ == '__main__':
	try:
		print('start server...')
		server_x = model_server.Model(26000)
		#server_y = model_server.Model(27000)
		print('servers setup completed')
		server_x.start()
		server_x.log_output(0)
		#server_y.start()
		while True:
			cm = input('$>')
			if len(cm)>0:
				msg1 = float(cm)
				#msg2 = float(cm[1])
				server_x.send_data(msg1)	
				#server_y.send_data(msg2)	
	except:
		traceback.print_exc()
