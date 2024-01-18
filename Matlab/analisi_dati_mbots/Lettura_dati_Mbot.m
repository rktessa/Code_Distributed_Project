clc;
clear all;
close all;

%% Lettura del file
% Mbot 1, 1000 righe per ogni robot
% 0-999 riferite a 1
% 1000-1999 riferite a 2 e così via
%fileID = fopen('C:\Users\rates\Downloads\Log_Mbots\Distributed\num_1\data_Mbot_12.txt','r');
fileID = fopen('..\Log_Mbots\Distributed\num_2\data_Mbot_12.txt','r');
%fileID = fopen('..\Log_Mbots\Rotazioni_della_1\data_Mbot_111.txt','r');
formatSpec = '%f';
sizeA = [12 Inf];

A = fscanf(fileID,formatSpec, sizeA);

%% Mbot1
time_1 = A(1,1:1000); % Ci sono dati fino a 880

acc_x_1 = A(2,1:1000)';
acc_y_1 = A(3:1000)';
acc_z_1 = A(4,1:1000)';

gyr_x_1 = A(5,1:1000)';
gyr_y_1 = A(6,1:1000)';
gyr_z_1 = A(7,1:1000)';

vel_linear_1 = A(8,1:1000)'; %cm/s
vel_omega_encoder_1 = A(9,1:1000)'; %rad/s
distance_1 = A(10,1:1000)'; %m

rpm_l_1 = A(11,1:1000)'; % rotations per min
rpm_r_1 = A(12,1:1000)'; % rotations per min

%% Mbot 2
time_2 = A(1,1001:2000)'; % Ci sono dati fino a 880

acc_x_2 = A(2,1001:2000)';
acc_y_2 = A(3, 1001:2000)';
acc_z_2 = A(4,1001:2000)';

gyr_x_2 = A(5,1001:2000)';
gyr_y_2 = A(6,1001:2000)';
gyr_z_2 = A(7,1001:2000)';

vel_linear_2 = A(8,1001:2000)'; %cm/s
vel_omega_encoder_2 = A(9,1001:2000)'; %rad/s
distance_2 = A(10,1001:2000)'; %m

rpm_l_2 = A(11,1001:2000)'; % rotations per min
rpm_r_2 = A(12,1001:2000)'; % rotations per min

%% Mbot 3
time_3 = A(1,2001:3000)'; % Ci sono dati fino a 880

acc_x_3 = A(2,2001:3000)';
acc_y_3 = A(3, 2001:3000)';
acc_z_3 = A(4,2001:3000)';

gyr_x_3 = A(5,2001:3000)';
gyr_y_3 = A(6,2001:3000)';
gyr_z_3 = A(7,2001:3000)';

vel_linear_3 = A(8,2001:3000)'; %cm/s
vel_omega_encoder_3 = A(9,2001:3000)'; %rad/s
distance_3 = A(10,2001:3000)'; %m

rpm_l_3 = A(11,2001:3000)'; % rotations per min
rpm_r_3 = A(12,2001:3000)'; % rotations per min

%% Mbot 4
time_4 = A(1,3001:4000)'; % Ci sono dati fino a 880

acc_x_4 = A(2,3001:4000)';
acc_y_4 = A(3, 3001:4000)';
acc_z_4 = A(4,3001:4000)';

gyr_x_4 = A(5,3001:4000)';
gyr_y_4 = A(6,3001:4000)';
gyr_z_4 = A(7,3001:4000)';

vel_linear_4 = A(8,3001:4000)'; %cm/s
vel_omega_encoder_4 = A(9,3001:4000)'; %rad/s
distance_4 = A(10,3001:4000)'; %m

rpm_l_4 = A(11,3001:4000)'; % rotations per min
rpm_r_4 = A(12,3001:4000)'; % rotations per min



%% Visualizzazione

fig = figure(1);
tiledlayout(2,1');
nexttile;
fontsize(fig, scale=1.2)
hold on
plot(time_2, gyr_x_2, 'r', 'LineWidth', 0.75)
plot(time_2, gyr_y_2, 'm', 'LineWidth', 0.75)
plot(time_2, gyr_z_2, 'b', 'LineWidth', 0.75)
legend('gyro_x','gyro_y', 'gyro_z')
xlabel('time [s]')
ylabel('angular velocity [rad/s]')
title('Gyroscope data')
hold off

nexttile;
fontsize(fig, scale=1.2)
hold on
plot(time_2, acc_x_2, 'r', 'LineWidth', 0.75)
plot(time_2, acc_y_2, 'm', 'LineWidth', 0.75)
plot(time_2, acc_z_2, 'b', 'LineWidth', 0.75)
legend('acc_x','acc_y', 'acc_z')
title('Accelerometer data')
xlabel('time [s]')
ylabel('acceleration [m/s^2]')
hold off


r = 0.032; %m raggio della ruota
L = 0.112; %m interasse ruote originale usato negli esperimenti
% Ricalcolo invertendo segno di una rpm velocità lineare e angolare
omega_l = rpm_l_2* (pi *2) /60;
omega_r = rpm_r_2* (pi *2) /60;

% Propagazione errore misura della velocità di rotazione calcolata 
% con le misure degli encoder
dr = 1/2 *1e-3; %m
dL = 0.5 * 1e-3; %m 

dw = sqrt( (dr/L * (omega_l-omega_r)).^2 + (r/L^2*(omega_l-omega_r)*dL).^2 );
R = L/2 * ((omega_l + omega_r)./(omega_r - omega_l));
%% Ricalcolo omega usando solo una misura degli encoder per capire 
% perch+ vedo offset nella misura

% Con due ruote che misurano
omega_calc = r/L * (omega_r - omega_l);
vel_cal_linear = r/2* (omega_r + omega_l)*100; %Corretta

fig = figure(2);
% fontsize(fig, scale=1.2)
tiledlayout('flow');

nexttile([1 1]);
hold on
% fontsize(fig, scale=1.2)
plot(time_2, omega_calc, 'LineWidth',1.2)
plot(time_2, gyr_z_2, 'LineWidth',1.2)
legend('omega calculated with encoder','omega measured by gyroscope')
title('Angular velocity omega ')
xlabel('time [s]')
ylabel('ang vel [rad/s]')
hold off

nexttile([1 1]);
hold on
% fontsize(fig, scale=1.2)
plot(time_2, vel_linear_2, 'LineWidth',1.2)
plot(time_2, vel_cal_linear, 'LineWidth',1.2)
xlabel('time [s]')
ylabel('velocity [cm/s]')
title('Linear velocity')
legend('vel computed during run', 'vel recalculated')
hold off

nexttile([1 2]);

hold on
fontsize(fig, scale=1.2)
plot(time_2, rpm_l_2, 'LineWidth',1.2)
plot(time_2, rpm_r_2, 'LineWidth',1.2)
title('rpm Mbots wheel')
xlabel('time [s]')
ylabel('RPM wheel [rpm]')
legend('left wheel', 'right wheel')
hold off





fig = figure(5);
hold on
fontsize(fig, scale=1.2)
plot(time_2, distance_1, 'LineWidth', 1) % distance between 1 and 1
plot(time_2, distance_2, 'LineWidth', 1) %distance between 2 and 1
plot(time_2, distance_3, 'LineWidth', 1)
plot(time_2, distance_4, 'LineWidth', 1)
legend('Distance 2 to 1', 'Distance 2 to 1','Distance 2 to 3', 'Distance 2 to 4' )
xlabel('time [s]')
ylabel('distance [m]')
title('Distances between Mbots')

delta_t = zeros(1,800);
for i = 1:800
    delta_t(i) = time_3(i+1)-time_3(i);
end
fig =figure(6)
fontsize(fig, scale=1.2)
plot(delta_t)
title('Frequency of data')