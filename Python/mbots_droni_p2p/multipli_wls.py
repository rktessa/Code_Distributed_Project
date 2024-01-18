
import numpy as np
import matplotlib.pyplot as plt

from wls_class import *

# Caso con 1 anchor e 4 Mbots

# Define Anchors
An = np.array([[0, 0]])

# Define sigma_error values in meters
sigma_error = np.array([np.sqrt(0.11), np.sqrt(0.22), np.sqrt(0.11), np.sqrt(0.22),
                        np.sqrt(0.23)])

# Mbot's true position
mbot_1 = np.array([5, 5])
mbot_2 = np.array([1, 1])
mbot_3 = np.array([7, 3])
mbot_4 = np.array([2, 9])

x_input = np.array([[5, 5],[1,1],[7,3],[2,9]])  # Empty if only one Mbot and 4 anchors are used

n_agents = 5



wls_1 = WLSClass(An, sigma_error, n_agents, x_input, mbot_1)
wls_2 = WLSClass(An, sigma_error, n_agents, x_input, mbot_2)
wls_3 = WLSClass(An, sigma_error, n_agents, x_input, mbot_3)
wls_4 = WLSClass(An, sigma_error, n_agents, x_input, mbot_4)


# Initialize the WLS1
dist = wls_1.distance(x_input, mbot_1)
z0 = wls_1.z_sensor(x_input, dist)    
x_est, P = wls_1.initialization(x_input, z0)
# Initialize the WLS2
dist = wls_2.distance(x_input, mbot_2)
z0 = wls_2.z_sensor(x_input, dist)    
x_est_2, P2 = wls_2.initialization(x_input, z0)
# Initialize the WLS1
dist = wls_3.distance(x_input, mbot_3)
z0 = wls_3.z_sensor(x_input, dist)    
x_est_3, P3 = wls_3.initialization(x_input, z0)
# Initialize the WLS1
dist = wls_4.distance(x_input, mbot_4)
z0 = wls_4.z_sensor(x_input, dist)    
x_est_4, P4 = wls_4.initialization(x_input, z0)


sol = np.zeros((100, 2))
sol[0] = [5.4, 5.2]

sol2 = np.zeros((100, 2))
sol2[0] = [1, 1]

sol3 = np.zeros((100, 2))
sol3[0] = [7, 3]

sol4 = np.zeros((100, 2))
sol4[0] = [2, 9]

# Initial position estimation
for i in range(1, 100):
    x_est, P = wls_1.WLS_distributed(x_est, P, x_input)
    x_est_2, P2 = wls_2.WLS_distributed(x_est_2, P2, x_input)
    x_est_3, P3 = wls_3.WLS_distributed(x_est_3, P3, x_input)
    x_est_4, P4 = wls_4.WuLS_distributed(x_est_4, P4, x_input)

    x_input = np.array([ [x_est[1],x_est[2]], [x_est_2[1],x_est_2[2]], [x_est_3[1],x_est_3[2]], [x_est_4[1],x_est_4[2]] ])
    sol[i]  = [ x_est[1], x_est[2] ] 
    sol2[i] = [ x_est_2[1],x_est_2[2] ] 
    sol3[i] = [ x_est_3[1],x_est_3[2] ] 
    sol4[i] = [ x_est_4[1],x_est_4[2] ] 
        
  
plt.figure()
plt.plot(sol[:, 0], 'b', linewidth=1.0)
plt.plot(sol[:, 1], 'b', linewidth=1.0)

plt.plot(sol2[:, 0], 'r', linewidth=1.0)
plt.plot(sol2[:, 1], 'r', linewidth=1.0)
plt.axhline(y=mbot_1[0], color='k', linestyle='-')
plt.axhline(y=mbot_2[0], color='k', linestyle='-')

plt.legend(['Mbot1_x', 'Mbot1_y', 'Reference'])
plt.title('Initial position estimation')




plt.show()
