import numpy as np
import time
import pickle

class Node:
    def __init__(self, port):
        self.leng = 1000
        self.port = port # the identity of my node, I use TCP for convenience
        self.time = 0 # Time from the last measure
        self.last_time = 0 # To check if the node is alive
        self.data = np.zeros((self.leng, 14)) #Inizialize a matrix to fill with all the datas from the mbots
        self.n = 0 # To register the n times the data is written
        
        self.data[0,12] =1 # valore per prima iterazione non nulla
        self.data[0,13] =1
        
        self.r = 6.4/2 # radius of the wheel in cm
        self.L = 11.2  # interasse in cm
    def update(self, time_mess, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, distance, x_pos, y_pos):
        
        # self.data[self.n,0] = float(time_mess)
        # self.time = float(time_mess)
        self.data[self.n,0] = time.perf_counter()
        
       
        self.data[self.n,1] = float(acc_x)
        self.data[self.n,2] = float(acc_y)
        self.data[self.n,3] = float(acc_z)

        self.data[self.n,4] = float(gyro_x)
        self.data[self.n,5] = float(gyro_y)
        self.data[self.n,6] = float(gyro_z)

        omega_l = float(rpm_l)*(np.pi *2) /60
        omega_r = float(rpm_r)*(np.pi *2) /60

        vel_linear = self.r/2 *(omega_l + omega_r) # cm/s
        omega_encoder = (self.r/self.L)*(omega_l - omega_r) # rad/s

        self.data[self.n,7] = vel_linear
        self.data[self.n,8] = omega_encoder

        self.data[self.n,9]  = distance
        self.data[self.n,10]  = rpm_l
        self.data[self.n,11]  = rpm_r

        self.data[self.n,12]  = x_pos
        self.data[self.n,13]  = y_pos
        #print(self.data[self.n,:])
        self.n = self.n + 1
        status = 1
        self.last_time = time.perf_counter()
        
        return status, self.n
    
    def check_is_alive(self):
        if (time.perf_counter() - self.last_time) >= 10: # se è più di 10 secondi che non riceve info lo dichiaro morto
            print("Node with port number %s is dead" % self.port)
            # Qui capire poi come escluderre da MDS usando quel nodo finchè non torna in vita?
            stop = 0
            return stop


def work(message):
    #print(message)
    # topic, port, time_mess, acc_x, acc_y, acc_z,  ==> Cosa mi aspetto di ricevere
    #cosa ricevo
    # b'Data,5562,Inizio,66.16,0.03,-0.12,0.12,-0.00,-0.01,0.01,62.67,96.41,Fine'
    try:
        topic, port,inizio, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, fine, x_pos,y_pos = message.split(",") 
        #print(message)
        return port, time_mess, accX, accY, accZ, gyro_x, gyro_y, gyro_z, rpm_l, rpm_r, x_pos,y_pos
    except Exception as e:
        print(e) # Se non è la stringa corretta non la leggo
        #print("Ho ricevuto qualcosa, ma non lo uso ")
        #print(message) # In questo modo visualizzo i messaggi inviati da Arduino e monitoro il suo funzionamento
        pass
    # If you want to use for math convert into float, these strings objects!    

class WLSClass_real:
    def __init__(self, anchors, sigma_error, n_agents, x_input):
        self.An = anchors
        self.sigma_e = sigma_error
        self.n_agents = n_agents
        self.Cz = np.diag(sigma_error[:n_agents]**2)
        self.x_input = x_input
        

    def initialization(self, x_input_est, z0):
        H = self.H_matrix(x_input_est)
        P = np.linalg.pinv(H.T @ np.linalg.inv(self.Cz) @ H)
        z = self.z_sensor(x_input_est, z0)
        x_est = P @ H.T @ np.linalg.inv(self.Cz) @ z0
        return x_est, P

    
    def WLS2_distributed(self, x_est, P, x_input_est, distances_measured):
        
        z = self.z_sensor(x_input_est, distances_measured)

        H = self.H_matrix(x_input_est)
        S = H @ P @ H.T + self.Cz
        W = P @ H.T @ np.linalg.pinv(S)
        x_est = x_est + W @ (z - H @ x_est)
        P = (np.eye(3) - W @ H) @ P
        return x_est, P
    
    def z_sensor(self, x_input_est, dist):
        row_An = self.An.shape[0]
        row_x = x_input_est.shape[0]
        z = np.zeros(row_An + row_x)

        for i in range(row_An):
            z[i] = dist[i]**2 - np.linalg.norm(self.An[i])**2

        for i in range(row_An, row_An + row_x):
            z[i] = dist[i]**2 - np.linalg.norm(x_input_est[i - row_An])**2

        return z

    def H_matrix(self, x_input_est):
        row_An = self.An.shape[0]
        row_x = x_input_est.shape[0]
        H = np.zeros((row_An + row_x, 3))
        H[:, 0] = 1

        for i in range(row_An):
            H[i, 1] = -2 * self.An[i, 0]
            H[i, 2] = -2 * self.An[i, 1]

        for j in range(row_An, row_An + row_x):
            H[j, 1] = -2 * x_input_est[j - row_An, 0]
            H[j, 2] = -2 * x_input_est[j - row_An, 1]

        return H



class WLSClass:
    def __init__(self, anchors, sigma_error, n_agents, x_input, mbot_position):
        self.An = anchors
        self.sigma_e = sigma_error
        self.n_agents = n_agents
        self.Cz = np.diag(sigma_error[:n_agents]**2)
        self.x_input = x_input
        self.mbot_position = mbot_position

    def initialization(self, x_input_est, z0):
        H = self.H_matrix(x_input_est)
        P = np.linalg.pinv(H.T @ np.linalg.inv(self.Cz) @ H)
        x_est = P @ H.T @ np.linalg.inv(self.Cz) @ z0
        return x_est, P

    def WLS_distributed(self, x_est, P, x_input_est):
        dist = self.distance(x_input_est, self.mbot_position)
        z = self.z_sensor(x_input_est, dist)

        H = self.H_matrix(x_input_est)
        S = H @ P @ H.T + self.Cz
        W = P @ H.T @ np.linalg.pinv(S)
        x_est = x_est + W @ (z - H @ x_est)
        P = (np.eye(3) - W @ H) @ P
        return x_est, P

    def WLS2_distributed(self, x_est, P, x_input_est, x_mbot):
        dist = self.distance(x_input_est, x_mbot)
        z = self.z_sensor(x_input_est, dist)

        H = self.H_matrix(x_input_est)
        S = H @ P @ H.T + self.Cz
        W = P @ H.T @ np.linalg.pinv(S)
        x_est = x_est + W @ (z - H @ x_est)
        P = (np.eye(3) - W @ H) @ P
        return x_est, P

    def H_matrix(self, x_input_est):
        row_An = self.An.shape[0]
        row_x = x_input_est.shape[0]
        H = np.zeros((row_An + row_x, 3))
        H[:, 0] = 1

        for i in range(row_An):
            H[i, 1] = -2 * self.An[i, 0]
            H[i, 2] = -2 * self.An[i, 1]

        for j in range(row_An, row_An + row_x):
            H[j, 1] = -2 * x_input_est[j - row_An, 0]
            H[j, 2] = -2 * x_input_est[j - row_An, 1]

        return H

    def z_sensor(self, x_input_est, dist):
        row_An = self.An.shape[0]
        row_x = x_input_est.shape[0]
        z = np.zeros(row_An + row_x)

        for i in range(row_An):
            z[i] = dist[i]**2 - np.linalg.norm(self.An[i])**2

        for i in range(row_An, row_An + row_x):
            z[i] = dist[i]**2 - np.linalg.norm(x_input_est[i - row_An])**2

        return z

    def distance(self, x_input_est, x_mbot):
        row_An = self.An.shape[0]
        row_x = x_input_est.shape[0]
        dist = np.zeros(row_An + row_x)
        #print(x_input_est)
        for l in range(row_An):
            dist[l] = np.linalg.norm(self.An[l] - x_mbot) + np.random.randn(1) * self.sigma_e[l]

        for l in range(row_An, row_An + row_x):
            #print(l)
            dist[l] = np.linalg.norm(x_input_est[l - row_An] - x_mbot) + np.random.randn(1) * self.sigma_e[l]
            #dist[l] = np.linalg.norm(x_input_est[l - row_An] - x_mbot) + np.random.randn(1) * self.sigma_e[l]
        return dist
    

    


def mbot_traj(r, L, x_0, y_0, phi_0):
    step = 0.05  # time step in seconds, 20 Hz measurements

    time = np.zeros(2000)
    time[0] = step
    x_pos = np.zeros(2000)
    x_pos[0] = x_0
    y_pos = np.zeros(2000)
    y_pos[0] = y_0
    phi = np.zeros(2000)
    phi[0] = phi_0
    rpm_r_v = np.zeros(2000)
    rpm_r_v[0] = 0
    rpm_l_v = np.zeros(2000)
    rpm_l_v[0] = 0

    stop_j = np.random.randint(50, 71)
    stop_k = np.random.randint(15, 31)
    j = 0
    k = 0
    for i in range(1, 2000):
        stop_j = np.random.randint(50, 71)
        stop_k = np.random.randint(15, 31)

        time[i] = step * i
        

        if j <= stop_j:
            rpm_r = 90
            rpm_l = rpm_r
            j += 1
        else:
            if k <= stop_k:
                rpm_r = np.random.randint(-50, 51)
                rpm_l = -rpm_r
                k += 1
            else:
                j = 0
                k = 0

        rpm_r_v[i] = rpm_r
        rpm_l_v[i] = rpm_l
        
        omega_l = (rpm_l * 2 * np.pi) / 60
        omega_r = (rpm_r * 2 * np.pi) / 60

        vel_linear = r / 2 * (omega_l + omega_r)  # cm/s
        omega = (r / L) * (omega_l - omega_r)  # rad/s

        p = phi[i - 1] + omega * step
        x = x_pos[i - 1] + vel_linear * step * np.cos(p)
        y = y_pos[i - 1] + vel_linear * step * np.sin(p)

        x_pos[i] = x
        y_pos[i] = y
        phi[i] = p
    
    matrix = np.array([time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v])    
    return matrix.T 


'''
Input functions:
    - sigma_error = numpy array containing standard deviation of measure done by UWB anchors
    - distances = numpy array containing distances
    '''

def WLS_static(sigma_error, distances):
# Define Anchors there is one that is fixed and I consider it is in the origin

    anchors = np.array([[0, 0]])