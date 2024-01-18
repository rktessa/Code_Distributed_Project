clc;
clear all;
close all;
%% Lettura dati giroscopio e comparazione con encoder

%Importazione dati
fileID = fopen('..Log_Mbots\Rotazioni_della_2\data_Mbot_rotation03.txt','r');
formatSpec = '%f';
sizeA = [12 Inf];
% Ordine dati
% time, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, vel_linear_encoder,
% vel_omega_encoder, distance, rpm_l, rpm_r
A = fscanf(fileID,formatSpec, sizeA);
data_lenght = length(A);
data_lenght = 400;
time = A(1,1:data_lenght)'; % Ci sono dati fino a 880

acc_x = A(2,1:data_lenght)';
acc_y = A(3,1:data_lenght)';
acc_z = A(4,1:data_lenght)';

gyr_x = smooth(A(5,1:data_lenght))'; %Smoothed
gyr_y = smooth(A(6,1:data_lenght))'; %
gyr_z = smooth(A(7,1:data_lenght))'; %

gyr_x = A(5,1:data_lenght)'; %Original
gyr_y = A(6,1:data_lenght)'; %
gyr_z = A(7,1:data_lenght)'; %

vel_linear = A(8,1:data_lenght)'; %cm/s
vel_omega_encoder = A(9,1:data_lenght)'; %rad/s
c = smooth(vel_omega_encoder(:))';
distance = A(10,1:data_lenght)'; %m



rpm_l = smooth(A(11,1:data_lenght)); % rotations per min
rpm_r = smooth(A(12,1:data_lenght)); % rotations per min

% Calcolo delle velocità angolari partendo dalle rpm dei motori
omega_l = rpm_l* (pi *2) /60;
omega_r = rpm_r* (pi *2) /60;

r = 0.032; %m raggio della ruota
L = 0.112; %m interasse ruote

% % Propagazione errore misura della velocità di rotazione calcolata 
% % con le misure degli encoder
% dr = 1/2 *1e-3; %m
% dL = 0.5 * 1e-3; %m 
% 
% dw = sqrt( (dr/L * (omega_l-omega_r)).^2 + (r/L^2*(omega_l-omega_r)*dL).^2 );
% 
R = L/2 * ((omega_l + omega_r)./(omega_r - omega_l));
% 
% % Ricalcolo omega usando solo una misura degli encoder per capire 
% % perch+ vedo offset nella misura

omega_calc = r/L * (omega_r - omega_l);

vel_cal_linear = r/2* (omega_l+omega_r)*100; %Corretta

% vel_cal_r = r/2* (2*omega_r)*100; % Errore nella lettura dell'encoder

% % Calcolo offset medio
% pippo = omega_l(1:45);
% offset  = sum(omega_l(72:250)+omega_r(72:250))/(179);
% offset2  = sum(omega_l(1:45)-omega_r(1:45))/(45);
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
plot( vel_cal_linear, 'r', 'LineWidth', 0.75) %cm/s
title('Linear velocity of Mbot')


i =i+1; 
figure(i)
hold on
plot( omega_calc, 'm', 'LineWidth', 0.75)
plot( gyr_z, 'b', 'LineWidth', 1)
%plot(time, c, 'k', 'LineWidth', 1)
%plot(time,omega_calc, 'r','LineWidth', 1)
%plot(time,omega_calc2, 'g','LineWidth', 1)
legend('omega encoder','omega gyro', 'encoder vel smoothed ', 'omega_calc', 'omega_calc2')
title('Angolar velocity of Mbot comparison encoder vs gyroscope')




i =i+1; 
figure(i)
hold on
plot( rpm_l, 'r', 'LineWidth', 0.75)
plot( rpm_r, 'b', 'LineWidth', 0.75)
legend('rpm_l','rpm_r')
title('Wheels rpm velocities')


i =i+1; 
figure(i)
plot(time, R, 'r', 'LineWidth', 0.75)
legend('R ICC')
title('Signed distance from the ICC')



