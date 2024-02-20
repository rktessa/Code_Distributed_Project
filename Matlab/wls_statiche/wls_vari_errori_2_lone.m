clc;
clear all;
close all;

%% WLS code solution for Mbots
% Code with only one Mbot and Anchors
% Anchors
A1 = [0,0];
A2 = [9.0, 2.0];
A3 = [1, 9.0];
%A4 = [0, 8.0];
anchors = [A1;A2;A3];
fig = 0;
iter = 80; %number iteration to calculate solutio
size_mag = 10; % varitaion of sigma error 
sol= zeros(iter,2, size_mag);

sigma_vec = zeros(size_mag,1);
E_sigma = zeros(size_mag,2); %error rmse for each value of uncertainty
V_sigma = zeros(size_mag,2);
for mag = 0: (size_mag-1)
    sigma_error = [sqrt(0.21+mag),...
                   sqrt(0.22+mag),...
                   sqrt(0.21+mag)]; %in meters
    sigma_vec(mag+1,1) = mean(sigma_error); 
    %Valore che voglio calcolare inizializzato qui ==> mbot_posizione_vera
    mbot_posizione_vera = [5,6];
    
    x_input = []; %se è vuoto sono nel caso 1 mbot e 4 ancore e basta
    
    n_agents= 3;
    % Inizializzo la classe
    wls_1 = wls_class(anchors, sigma_error,n_agents,x_input, mbot_posizione_vera, mbot_posizione_vera); %n_agents è il numero di agenti, va specificato(n ancore+ nmbot)
   
    % La funzione distance serve solo nella simulazione, poi sarà sostituita da
    % lettura UWB
    dist = wls_1.distance(x_input,mbot_posizione_vera); %inizializzo la prima
    z0 = wls_1.z_sensor(x_input, dist);
    [x_est, P] = wls_1.initialization(x_input,z0);
    
    
    sol(1,:, mag+1)= [x_est(2),x_est(3)];
    
    % Prime iterazioni con algoritmo per stimare posizioni iniziali
    for i=2:iter        
        dist = wls_1.distance(x_input,mbot_posizione_vera);
        [x_est, P] = wls_1.WLS3_distributed(x_est, P, x_input, dist);
        x_input = [];
        sol(i,:,mag+1) = [x_est(2), x_est(3)]; % salvo le soluzioni  
    end
    % Error calculation
    F1 = sol(:,1,mag+1);
    F2 = sol(:,2,mag+1);
    A1 = mbot_posizione_vera(1);
    A2 = mbot_posizione_vera(2);
    % https://it.mathworks.com/matlabcentral/answers/4064-rmse-root-mean-square-error
    E_sigma(mag+1,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_sigma(mag+1,2) = sqrt(mean((A2 - F2).^2));
    V_sigma(mag+1,1) = var(F1);
    V_sigma(mag+1,2) = var(F2);

end




%% Plotting the results
%Plot initial position estimation
    Mbot_1 = [x_est(2), x_est(3)];  % takes the last steps 
    
    
    fig = fig+1;
    figure(fig)
    tiledlayout(2,1)

    nexttile
    hold on
    plot(sol(:,1,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,1,4), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,1,7), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,1,10), 'linewidth',1.0 )
    yline(mbot_posizione_vera(1), 'd')
    str1 = sprintf('err_1 = %f',sigma_vec(1));
    str2 = sprintf('err_4 = %f',sigma_vec(4));
    str3 = sprintf('err_7 = %f',sigma_vec(7));
    str4 = sprintf('err_{10} = %f',sigma_vec(10));
    legend(str1, str2, str3,str4)
    title('Evolution of solution calculation')
    ylabel('x position [m]')
    xlabel('iterations')
    hold off

    nexttile
    hold on
    plot(sol(:,2,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,2,4), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,2,7), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,2,10), 'linewidth',1.0 )
    yline(mbot_posizione_vera(2), 'd')
    ylabel('y position [m]')
    xlabel('iterations')
    hold off

    % Plotting RMSE
    fig = fig+1;
    figure(fig)
    tiledlayout(2,1)
    nexttile
    hold on
    plot(sigma_vec.^2,E_sigma(:,1), 'LineWidth',1.5)
    plot(sigma_vec.^2,E_sigma(:,2), 'LineWidth',1.5)
    legend('RMSE x axis', 'RMSE y axis')
    xlabel('RMSE [m]')
    ylabel('uncertainty [m]')
    hold off
    nexttile
    plot(sigma_vec,V_sigma(:,1), 'LineWidth',1.5)
    plot(sigma_vec,V_sigma(:,2), 'LineWidth',1.5)
    legend('variance x axis', 'variance y axis')
%% Multiple simulation for having an histogram 


% Faccio x volte la simulazione per avere una media del RMSE
n = 10;
E_multiple = zeros(n,1); % n = number of simulations
var_multiple = zeros(n,1);
sol_multiple = zeros(n,2); % First element of the solution
for e = 1:n
    %% Re initialize the code to compute the position of moving Mbot with WLS
    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),...
                   sqrt(0.21)]; %in meters
    %randomize the position
    x_input = [];
    mbot_vera_second = randi([0,5],1,2);

    wls_mult = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second, mbot_vera_second);
    dist = wls_mult.distance(x_input,mbot_vera_second); %inizializzo la prima
    z0 = wls_mult.z_sensor(x_input, dist);
    [x_est, P] = wls_mult.initialization(x_input,z0);
    
    
    sol_multiple(1,:)= [x_est(2),x_est(3)];
    for l=2:1000
        dist = wls_mult.distance(x_input,mbot_vera_second);
        [x_est, P] = wls_mult.WLS3_distributed(x_est, P, x_input, dist);
        x_input = [];
        sol_multiple(l,:) = [x_est(2), x_est(3)]; % salvo le soluzioni
    end
    %  Error calculation
    
    F1 = sol_multiple(:,1);
    F2 = sol_multiple(:,2);
    A1 = mbot_vera_second(1);
    A2 = mbot_vera_second(2);
    E_multiple(e,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,2) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,1) = var(F1);
    var_multiple(e,2) = var(F2);

    %E(e) = sqrt(mean((A - F).^2));
end

 %% Plot histogram
fig = fig+1;
figure(fig)
hold on
h = histogram(E_multiple(:,1));
h2 = histogram(E_multiple(:,2));
legend('RMSE x_{pos}', 'RMSE y_{pos}')
ylabel('frequency')
xlabel('RMSE [m]')
title('RMSE of 1000 simulations with one agent and three anchors')

Mean_RMSE_static = mean(E_multiple);
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% Moving Mbot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To simulate the motion, a differential drive model is simulated, varying
% rpm of wheels
r2 = 6.4/2 *0.01; % radius of the wheel in cm
L2 = 11.2 *0.01;  % interasse in cm




% Faccio x volte la simulazione per avere una media del RMSE
n = 10;
E_movement = zeros(n,1); % n = number of simulations
var_movement = zeros(n,1); 
for e = 1:n
    % initial conditions
    x_02 = randi([0,5],1);
    y_02 = randi([0,5],1);
    phi_02 = -3.14 + (3.14+3.14)*rand(1,1); %Initial angle in rad
    % This create the motion of the Mbot on a random trajectory
    Mbot_data = mbot_traj(r2, L2, x_02, y_02, phi_02); % output shape [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v]
    
    % Simulation of WLS computation with moving Mbot
    % Now calculate at each time the estimate made by the wls recursive
    sol_movement = zeros(2000,2);
    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),...
                   sqrt(0.21)]; %in meters

    % Re initialize the code to compute the position of moving Mbot with WLS
    mbot_vera_motion = [x_02, y_02];
    
    wls_movement = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second, mbot_vera_second);

    dist = wls_movement.distance(x_input,mbot_vera_motion); %inizializzo la prima
    z0 = wls_movement.z_sensor(x_input, dist);
    [x_est, P] = wls_movement.initialization(x_input,z0);

    x_mbot = [Mbot_data(1,2), Mbot_data(1,3)]; % The initial position used to calculate distance in simulation
    sol_movement(1,:) = [x_est(2), x_est(3)]; % First element of the solution
    for l2=2:2000
        x_mbot = [Mbot_data(l2,2), Mbot_data(l2,3)];
        
        if mod(l2,40)== 0 %ogni x misure reinizializzo, 20 sembra un minimo con questa dinamica di Movimento di Mbot
            dist = wls_movement.distance(x_input,x_mbot); %reinizializzo
            z0 = wls_movement.z_sensor(x_input, dist);
            [x_est, P] = wls_movement.initialization(x_input,z0);
        end

        %at each iteration the position change
        dist = wls_movement.distance(x_input,x_mbot);
        [x_est, P] = wls_movement.WLS3_distributed(x_est, P, x_input, dist);
        x_input = [];
        sol_movement(l2,:) = [x_est(2), x_est(3)];
    end
    % Error calculation
    F1 = sol_movement(:,1);
    F2 = sol_movement(:,2);
    A1 = Mbot_data(:,2);
    A2 = Mbot_data(:,3);
    E_movement(e,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_movement(e,2) = sqrt(mean((A2 - F2).^2));
    var_movement(e,1) = var(A1-F1);
    var_movement(e,2) = var(A2-F2);

end

% variance hist
i = i+1;
figure(i)
hold on
h3 = histogram(var_movement(:,1));
h4 = histogram(var_movement(:,2));
ylabel('frequenza')
xlabel('variance')
title('variance of 1000 simulation with one moving agent and three anchors')
legend('var x axis', 'var y axis')
hold off

% rmse hist
i = i+1;
figure(i)
hold on
h3 = histogram(E_movement(:,1));
h4 = histogram(E_movement(:,2));
ylabel('frequenza')
xlabel('RMSE')
title('RMSE of 1000 simulation with one moving agent and three anchors')
legend('RMSE x axis', 'RMSE y axis')
hold off
Mean_RMSE_motion = mean(E_movement)

% Plot the figure

fig = fig+1;
figure(fig)
hold on
plot(sol_movement(:,1), sol_movement(:,2), '*')
plot(Mbot_data(:,2),Mbot_data(:,3), 'LineWidth',1.5)
% plot(sol_movement(:,3), sol_movement(:,4), '*b')
% plot(sol_movement(:,5), sol_movement(:,6), 'og')
xlabel('x axis [m]')
ylabel('y axis [m]')
title('Mbot trajectory estimation')
legend( 'estimated position','ideal trajectory')
%% Animated plot  
fig = fig+1;
figure(fig)
hold on
h = animatedline('Color','b','LineWidth',0.75);
g = animatedline('Color','r','LineStyle', ':');
A = Mbot_data(:,2); B = Mbot_data(:,3);
C = sol_movement(:,1); D = sol_movement(:,2);
for k = 1:length(A)
    addpoints(h,A(k),B(k))
    addpoints(g,C(k),D(k))
     % xlim([min(C) max(C)]);
     % ylim([min(D) max(D)]);
    drawnow limitrate
    pause(0.005)
end
hold off
%% Function that generate Mbot motion
function matrix = mbot_traj(r, L, x_0, y_0, phi_0)
    step = 0.05; %time step in seconds, 20 Hz measurements

    time =   zeros(2000,1); time(1) = step;
    x_pos = zeros(2000,1); x_pos(1) = x_0;
    y_pos = zeros(2000,1); y_pos(1) = y_0;
    phi = zeros(2000,1); phi(1) = phi_0;
    rpm_r_v = zeros(2000,1); rpm_r_v(1) = 0;
    rpm_l_v = zeros(2000,1); rpm_l_v(1) = 0;

    stop_j = randi([50, 70], 1);
    stop_k = randi([15, 30], 1);
    j = 0;
    k = 0;
    for i = 2:2000
        stop_j = randi([50, 70], 1);
        stop_k = randi([15, 30], 1);
        %print("dentro = ", i)
        time(i) = step*i;

        if j <= stop_j
            rpm_r = 90;
            rpm_l = rpm_r;
            j = j+1;
        else 
            if k <= stop_k   
                rpm_r = randi([-50,50],1); 
                rpm_l = -rpm_r;
                k = k +1;
            else 
                j = 0;
                k = 0;
            end
        end
        rpm_r_v(i) = rpm_r;
        rpm_l_v(i)= rpm_l;

        omega_l = (rpm_l)*(pi *2) /60;
        omega_r = (rpm_r)*(pi *2) /60;

        vel_linear = r/2 *(omega_l + omega_r); % cm/s
        omega = (r/L)*(omega_l - omega_r); % rad/s

        % ! Sto integrando, spero abbia un senso 
        p = (phi(i-1)) + omega*step;
        x = (x_pos(i-1)) + vel_linear *step * cos(p);
        y = (y_pos(i-1)) + vel_linear *step * sin(p);

        x_pos(i) = x;
        y_pos(i) = y;
        phi(i) = p;
     end

     matrix = [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v];

end
