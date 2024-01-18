import numpy as np
import matplotlib.pyplot as plt

from wls_class import *
    # ... (previously defined class methods)


# Define Anchors
anchors = np.array([[0, 0], [10, 0], [0, 10], [10, 10], [15, 20], [5, 20]])

# Define sigma_error values in meters
sigma_error = np.array([np.sqrt(0.11), np.sqrt(0.22), np.sqrt(0.11), np.sqrt(0.22),
                        np.sqrt(0.23), np.sqrt(0.21), np.sqrt(0.07), np.sqrt(0.37),
                        np.sqrt(0.31)])

# Mbot's true position
mbot_posizione_vera = np.array([5, 5])

x_input = np.array([])  # Empty if only one Mbot and 4 anchors are used

n_agents = 6
# Initialize the class
wls_1 = WLSClass(anchors, sigma_error, n_agents, x_input, mbot_posizione_vera)

# Initialize the WLS
dist = wls_1.distance(x_input, mbot_posizione_vera)
z0 = wls_1.z_sensor(x_input, dist)  #! questo fondamentale
x_est, P = wls_1.initialization(x_input, z0)

sol = np.zeros((100, 2))
sol[0] = [5.4, 5.2]

# Initial position estimation
for i in range(1, 100):
    x_est, P = wls_1.WLS_distributed(x_est, P, x_input)
    x_input = np.array([])
    sol[i] = [x_est[1], x_est[2]]

# Plot initial position estimation
Mbot_1 = [x_est[1], x_est[2]]


# ... (Previous code with the WLSClass definition)

# Moving Mbot simulation translated to Python
r2 = 6.4 / 2 * 0.01  # radius of the wheel in cm
L2 = 11.2 * 0.01     # interasse in cm
x_02 = 2
y_02 = 2
phi_02 = 0  # Initial angle in rad

# Generating Mbot's motion trajectory
Mbot_data = mbot_traj(r2, L2, sol[-1, 0], sol[-1, 1], phi_02)  # output shape [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v]


sol_movement = np.zeros((2000, 2))
sol_movement[0] = [sol[-1, 0], sol[-1, 1]]  # First element of the solution

n = 10
E = np.zeros(n)

for e in range(n):
    mbot_vera_second = [sol[-1, 0], sol[-1, 1]]
    dist = wls_1.distance(x_input, mbot_vera_second)
    z0 = wls_1.z_sensor(x_input, dist)
    x_est, P = wls_1.initialization(x_input, z0)
    
    x_mbot = [Mbot_data[0, 1], Mbot_data[0, 2]]  # Initial position used for distance calculation

    for l in range(1, 2000):
        if l % 20 == 0:
            dist = wls_1.distance(x_input, x_mbot)
            z0 = wls_1.z_sensor(x_input, dist)
            x_est, P = wls_1.initialization(x_input, z0)

        x_est, P = wls_1.WLS2_distributed(x_est, P, x_input, x_mbot)
        x_mbot = [Mbot_data[l, 1], Mbot_data[l, 2]]
        x_input = np.array([])
        sol_movement[l] = [x_est[1], x_est[2]]

    F = np.sqrt(sol_movement[:, 0] ** 2 + sol_movement[:, 1] ** 2)
    A = np.sqrt(Mbot_data[:, 1] ** 2 + Mbot_data[:, 2] ** 2)
    E[e] = np.sqrt(np.mean((A - F) ** 2))

Mean_RMSE = np.mean(E)
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
plt.show()





