import numpy as np
# Specify port and IP address for connections


port_0 = "5556"
port_1 = "5567"
port_2 = "5558"
port_3 = "5560"
port_4 = "5562"

port_list = [port_0,port_1,port_2,port_3,port_4]

ip_address_local = "localhost"
ip_address_0 = "192.168.123.23"
ip_address_1 = "192.168.123.77"
ip_address_2 = "192.168.123.9"
ip_address_3 = "192.168.123.5"
ip_address_4 = "192.168.123.6"

ip_list = [ip_address_0,ip_address_1,ip_address_2,ip_address_3,ip_address_4]

sigma_error = np.array([np.sqrt(0.5), np.sqrt(0.51), np.sqrt(0.52), np.sqrt(0.53), np.sqrt(0.54)])
