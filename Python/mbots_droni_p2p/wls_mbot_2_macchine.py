import numpy as np
import matplotlib.pyplot as plt

from wls_class import *
    # ... (previously defined class methods)





# Define Anchors
anchors = np.array([[0, 0], [10, 0], [0, 10], [10, 10], [15, 20], [5, 20]])
anchors = np.array([ [0, 0], [10, 0] ])

# Define sigma_error values in meters
sigma_error = np.array([np.sqrt(0.11), np.sqrt(0.22), np.sqrt(0.11), np.sqrt(0.22),
                        np.sqrt(0.23), np.sqrt(0.21), np.sqrt(0.07), np.sqrt(0.37)])

# Mbot's true position
mbot_posizione_vera = np.array([5, 5])
mbot_vera_2 = np.array([2, 7])

x_input = np.array([[1,1],[2,3]])  # Empty if only one Mbot and 4 anchors are used

n_agents = 4
# Initialize the class
wls_1 = WLSClass(anchors, sigma_error, n_agents, x_input, mbot_posizione_vera)
wls_2 = WLSClass(anchors, sigma_error, n_agents, x_input, mbot_vera_2)

# Initialize the WLS
dist = wls_1.distance(x_input, mbot_posizione_vera)
z0 = wls_1.z_sensor(x_input, dist)  #! questo è ondam
x_est, P = wls_1.initialization(x_input, z0)

dist2 = wls_2.distance(x_input, mbot_vera_2)
z02 = wls_2.z_sensor(x_input, dist2)  #! questo è ondam
x_est2, P2 = wls_2.initialization(x_input, z02)


it = 1000
sol = np.zeros((it, 4))
sol[0] = [x_est[0], x_est[1], x_est2[0], x_est2[1]]

# Initial position estimation
for i in range(1, it):
    x_est, P = wls_1.WLS_distributed(x_est, P, x_input)
    x_est2, P2 = wls_2.WLS_distributed(x_est2, P2, x_input)
    
    #x_input = np.array([[x_est[0], x_est[1]],[ x_est2[0], x_est2[1] ] ])
    x_input = np.array([[1,1],[ 1,1 ] ])

    sol[i] = [x_est[1], x_est[2], x_est2[1], x_est2[2]]

# Plot initial position estimation
Mbot_1 = [x_est[1], x_est[2]]
Mbot_2 = [x_est2[1], x_est2[2]]
print(Mbot_1)
print(Mbot_2)
# ... (Previous code with the WLSClass definition)

# Moving Mbot simulation translated to Python
r2 = 6.4 / 2 * 0.01  # radius of the wheel in cm
L2 = 11.2 * 0.01     # interasse in cm
x_02 = 2
y_02 = 2
phi_02 = 0  # Initial angle in rad
phi_03 = 3


# Generating Mbot's motion trajectory
Mbot_data = mbot_traj(r2, L2, sol[-1, 0], sol[-1, 1], phi_02)  # output shape [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v]
Mbot_data2 = mbot_traj(r2, L2, sol[-1, 2], sol[-1, 3], phi_03)  # output shape [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v]

sol_movement = np.zeros((2000, 4))
sol_movement[0] = [sol[-1, 0], sol[-1, 1], sol[-1, 2], sol[-1, 3]]  # First element of the solution



n = 100
E = np.zeros((n,2))

for e in range(n):
    mbot_vera_second = [sol[-1, 0], sol[-1, 1]]
    mbot_vera_2 = [sol[-1, 2], sol[-1, 3]]
    
    dist = wls_1.distance(x_input, mbot_vera_second)
    z0 = wls_1.z_sensor(x_input, dist)
    x_est, P = wls_1.initialization(x_input, z0)
    
    
   
    dist2 = wls_2.distance(x_input, mbot_vera_2)
    z02 = wls_2.z_sensor(x_input, dist2)
    x_est2, P2 = wls_2.initialization(x_input, z02)
    
    x_mbot = [Mbot_data[0, 1], Mbot_data[0, 2]]  # Initial position used for distance calculation
    x_mbot2 = [Mbot_data2[0, 1], Mbot_data2[0, 2]]
    for l in range(1, 2000):
        if l % 20 == 0:
            dist = wls_1.distance(x_input, x_mbot)
            z0 = wls_1.z_sensor(x_input, dist)
            x_est, P = wls_1.initialization(x_input, z0)
            
            dist2 = wls_2.distance(x_input, x_mbot2)
            z02 = wls_2.z_sensor(x_input, dist2)
            x_est2, P2 = wls_2.initialization(x_input, z02)

        x_est, P = wls_1.WLS2_distributed(x_est, P, x_input, x_mbot)
        x_est2, P2 = wls_2.WLS2_distributed(x_est2, P2, x_input, x_mbot2)
        
        x_mbot = [Mbot_data[l, 1], Mbot_data[l, 2]]
        x_mbot2 = [Mbot_data2[l, 1], Mbot_data2[l, 2]]
        
        #x_input = np.array([[1,2], [3,5] ])
        x_input = np.array([[x_est[1], x_est[2]],[ x_est2[1], x_est2[2] ] ])
        sol_movement[l] = [x_est[1], x_est[2], x_est2[1], x_est2[2]]

    F = np.sqrt(sol_movement[:, 0] ** 2 + sol_movement[:, 1] ** 2)
    A = np.sqrt(Mbot_data[:, 1] ** 2 + Mbot_data[:, 2] ** 2)
    E[e,0] = np.sqrt(np.mean((A - F) ** 2))
    
    F = np.sqrt(sol_movement[:, 2] ** 2 + sol_movement[:, 3] ** 2)
    A = np.sqrt(Mbot_data2[:, 1] ** 2 + Mbot_data2[:, 2] ** 2)
    E[e,1] = np.sqrt(np.mean((A - F) ** 2))

Mean_RMSE = np.mean(E[:,0])
Mean_RMSE_2 = np.mean(E[:,1])
print(Mean_RMSE)

# Plotting figures
plt.figure()
plt.hist(E)
plt.ylabel('Frequency')
plt.xlabel('RMSE')
plt.title('RMSE of 1000 simulations with one Mbot and 6 anchors')


plt.figure()
plt.plot(Mbot_data[:, 1], Mbot_data[:, 2], 'b')
plt.plot(sol_movement[:, 0], sol_movement[:, 1], ':r')
plt.plot(Mbot_data2[:, 1], Mbot_data2[:, 2], 'b')
plt.plot(sol_movement[:, 2], sol_movement[:, 3], ':r')

plt.show()




