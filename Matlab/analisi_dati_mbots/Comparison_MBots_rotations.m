clc;
clear all;
close all;
%% Lettura dati giroscopio e comparazione con encoder

%Importazione dati
fileID1 = fopen('..\Log_Mbots\Rotazioni_della_3\data_Mbot_rotation03.txt','r');
fileID3 = fopen('..\Log_Mbots\Rotazioni_della_3\data_Mbot_rotation13.txt','r');
formatSpec = '%f';
sizeA = [12 Inf];
% Ordine dati
% time, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, vel_linear_encoder,
% vel_omega_encoder, distance, rpm_l, rpm_r
A = fscanf(fileID1,formatSpec, sizeA);
C = fscanf(fileID3,formatSpec, sizeA);

data_lenght = length(A);
data_lenght = 400;

% Mbot 1
gyr_x1 = A(5,1:data_lenght)'; %Original
gyr_y1 = A(6,1:data_lenght)'; %
gyr_z1 = A(7,1:data_lenght)'; %

vel_linear1 = A(8,1:data_lenght)'; %cm/s
vel_omega_encoder1 = A(9,1:data_lenght)'; %rad/s


rpm_l1 = A(11,1:data_lenght); % rotations per min
rpm_r1 = A(12,1:data_lenght); % rotations per min

% Mbot 3
gyr_x3 = C(5,1:data_lenght)'; %Original
gyr_y3 = C(6,1:data_lenght)'; %
gyr_z3 = C(7,1:data_lenght)'; %

vel_linear3 = C(8,1:data_lenght)'; %cm/s
vel_omega_encoder3 = C(9,1:data_lenght)'; %rad/s
rpm_l3 = C(11,1:data_lenght); % rotations per min
rpm_r3 = C(12,1:data_lenght); % rotations per min

%% Calcoli per ora trascuro

% Calcolo delle velocità angolari partendo dalle rpm dei motori
omega_l1 = rpm_l1* (pi *2) /60;
omega_r1 = rpm_r1* (pi *2) /60;

r = 0.032; %m raggio della ruota
L = 0.112; %m interasse ruote

% % Propagazione errore misura della velocità di rotazione calcolata 
% % con le misure degli encoder
% dr = 1/2 *1e-3; %m
% dL = 0.5 * 1e-3; %m 
% 
% dw = sqrt( (dr/L * (omega_l-omega_r)).^2 + (r/L^2*(omega_l-omega_r)*dL).^2 );
% 
% R = L/2 * ((omega_l + omega_r)./(omega_r - omega_l));
% 
% % Ricalcolo omega usando solo una misura degli encoder per capire 
% % perch+ vedo offset nella misura

omega_calc_l1 = r/L * (2*omega_l1);
omega_calc_r1 = -r/L * (2*omega_r1);

vel_cal_l = r/2* (2*omega_l1)*100; %Corretta

vel_cal_r = r/2* (2*omega_r1)*100; % Errore nella lettura dell'encoder

% % Calcolo offset medio
% pippo = omega_l1(1:45);
% offset  = sum(omega_l1(72:250)+omega_r(72:250))/(179);
% offset2  = sum(omega_l1(1:45)-omega_r(1:45))/(45);
% % ricacolo la omega sbagliata togliendo l'offset
% omega_r = omega_r - offset;
% 
% omega_calc2 = r/L * (omega_l- omega_r); % La velocità angolare ricalcolata così è giusta.

%% Visualizzazione
i = 0;
% Velocità lineare

i =i+1; 
figure(i)
hold on
plot( vel_linear1, 'r', 'LineWidth', 0.75) %cm/s
plot( vel_linear3, 'b', 'LineWidth', 0.75) %cm/s
legend('vel linear 1','vel linear 3')
title('Linear velocity of Mbot')


i =i+1; 
figure(i)
hold on
plot( smooth(vel_omega_encoder1), 'r', 'LineWidth', 0.75)
plot( gyr_z1, 'b', 'LineWidth', 1)
plot( smooth(vel_omega_encoder3), 'm', 'LineWidth', 0.75)
plot( gyr_z3, 'g', 'LineWidth', 1)

plot( smooth(omega_calc_l1), 'k', 'LineWidth', 0.75)
plot( smooth(omega_calc_r1), 'y', 'LineWidth', 0.75)

legend('omega encoder 1','omega gyro 1', 'omega encoder 3','omega gyro 3', 'omega_calcleft1', 'omega_calc right 1')
title('Angolar velocity of Mbot comparison encoder vs gyroscope')




i =i+1; 
figure(i)
hold on
plot( rpm_l1, 'r', 'LineWidth', 0.75)
plot( rpm_r1, 'b', 'LineWidth', 0.75)
plot( rpm_l3, 'm', 'LineWidth', 1)
plot( rpm_r3, 'g', 'LineWidth', 1)
legend('rpm_l1','rpm_r1', 'rpm_l3','rpm_r3')
title('Wheels rpm velocities')



