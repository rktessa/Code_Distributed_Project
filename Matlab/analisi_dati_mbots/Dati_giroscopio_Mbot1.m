clc;
clear all;
close all;
%% Lettura dati giroscopio e comparazione con encoder

%Importazione dati
fileID = fopen('..\Log_Mbots\Rotazioni_della_1\data_Mbot_rotation30.txt','r');
fileID2 = fopen('..\Log_Mbots\Rotazioni_della_1\data_Mbot_rotation32.txt','r');
formatSpec = '%f';
sizeA = [12 Inf];
% Ordine dati
% time, acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, vel_linear_encoder,
% vel_omega_encoder, distance, rpm_l, rpm_r
A = fscanf(fileID,formatSpec, sizeA);
B = fscanf(fileID2,formatSpec, sizeA);

data_lenght = length(A);
data_lenght = 310;
time = A(1,1:data_lenght)'; % Ci sono dati fino a 880

acc_x = A(2,1:data_lenght)';
acc_y = A(3,1:data_lenght)';
acc_z = A(4,1:data_lenght)';

gyr_x = A(5,1:data_lenght)'; %Original
gyr_y = A(6,1:data_lenght)'; %
gyr_z = A(7,1:data_lenght)'; %

vel_linear = A(8,1:data_lenght)'; %cm/s
vel_omega_encoder = A(9,1:data_lenght)'; %rad/s
c = smooth(vel_omega_encoder(:))';
distance = A(10,1:data_lenght)'; %m



rpm_l = smooth(A(11,1:data_lenght)); % rotations per min
rpm_r = smooth(A(12,1:data_lenght)); % rotations per min

rpm_l2 = smooth(B(11,1:data_lenght)); % rotations per min
rpm_r2 = smooth(B(12,1:data_lenght)); % rotations per min

% Calcolo delle velocità angolari partendo dalle rpm dei motori
omega_l = rpm_l* (pi *2) /60;
omega_r = rpm_r* (pi *2) /60;

omega_l2 = rpm_l2* (pi *2) /60;
omega_r2 = rpm_r2* (pi *2) /60;


r = 0.032; %m raggio della ruota
L = 0.112; %m interasse ruote originale usato negli esperimenti


% Propagazione errore misura della velocità di rotazione calcolata 
% con le misure degli encoder
dr = 1/2 *1e-3; %m
dL = 0.5 * 1e-3; %m 

dw = sqrt( (dr/L * (omega_l-omega_r)).^2 + (r/L^2*(omega_l-omega_r)*dL).^2 );

R = L/2 * ((omega_l + omega_r)./(omega_r - omega_l));

%% Ricalcolo omega usando solo una misura degli encoder per capire 
% perch+ vedo offset nella misura

% Con due ruote che misurano
omega_calc = -r/L * (omega_r - omega_l);
vel_cal_linear = r/2* (omega_r + omega_l)*100; %Corretta
%Con misura solo ruota sinistra
omega_calc = -r/L * ( 2* omega_l);
vel_cal_linear = r/2* (omega_l + omega_l)*100; 
%Con misura solo ruota destra
omega_calc2 = -r/L * ( -2* omega_r2);
vel_cal_linear2 = r/2* (omega_r2 + omega_r2)*100; 


vel_cal_r = r/2* (2*omega_r)*100; % Errore nella lettura dell'encoder

%% Calcolo valore rotazione angolare con set rpm ruote

rpm_l_t = 54;
rpm_r_t = -54;

omega_l_t = rpm_l_t* (pi *2) /60;
omega_r_t = rpm_r_t* (pi *2) /60;


vel_omega_t = r/L * (omega_r_t - omega_l_t);



%FPS
fps = 130; %frame per fare un giro
video = 29.97;
time_giro = fps/video;
vel_rotazione = 2*pi/time_giro

%% Visualizzazione
i = 0;
% Velocità lineare

i =i+1; 
figure(i)
hold on
plot( vel_cal_linear, 'r', 'LineWidth', 0.75) %cm/s
legend('vel_linear')
title('Linear velocity of Mbot')


i =i+1; 
figure(i)
hold on
plot( omega_calc2, 'm', 'LineWidth', 0.75)
plot( gyr_z, 'b', 'LineWidth', 1)
plot(omega_calc, 'k','LineWidth', 0.75)
%plot(time,omega_calc2, 'g','LineWidth', 1)
legend('omega vel ricalc 2','omega gyro', 'encoder vel ricalc ')
title('Angolar velocity of Mbot comparison encoder vs gyroscope')




i =i+1; 
figure(i)
hold on
plot( rpm_l, 'r', 'LineWidth', 0.75)
plot( rpm_r, 'b', 'LineWidth', 0.75)
plot( rpm_l2, 'm', 'LineWidth', 0.75)
plot( rpm_r2, 'k', 'LineWidth', 0.75)
legend('rpm_l','rpm_r')
title('Wheels rpm velocities')



i =i+1; 
figure(i)
plot(time, R, 'r', 'LineWidth', 0.75)
legend('R ICC')
title('Signed distance from the ICC')

i =i+1; 
figure(i)
hold on
plot( (omega_calc-gyr_z), 'r', 'LineWidth', 0.75)
legend('Error between encoder and gyroscope')
title('Residuals')





