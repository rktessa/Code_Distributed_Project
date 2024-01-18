%% Simulation with adding latency in the comunincation
clc;
clear all;
close all;

% %% WLS code solution for Mbots
% % Code with 2 Mbot and 3 Anchors
% % Anchors
A1 = [0,0];
A2 = [9.0, 2.0];
A3 = [1, 9.0];
A4 = [0, 8.0];
anchors = [A1;A2;A3];
fig = 0;
iter = 80; %number iteration to calculate solutio
size_mag = 100; % varitaion of latency 
sol= zeros(iter,4, size_mag);

latency_vec = zeros(size_mag,2);
E_sigma = zeros(size_mag,4); %error rmse for each value of uncertainty
V_sigma = zeros(size_mag,4);

for con = 1: size_mag
    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),...
                   sqrt(0.22),...
                   sqrt(0.21),...
                   sqrt(0.21)]; %in meters

    min_a = con; 
    max_b = con + 4*con;
    agg = random_n(min_a, max_b);  b1 = 0;
    agg2 = random_n(min_a, max_b); k1 =0;
    latency_vec(con,:) = [min_a, max_b];

    %Valore che voglio calcolare inizializzato qui ==> mbot_posizione_vera
    mbot_posizione_vera = [5,6];
    mbot_posizione_vera2 = [4,7];
    reference_positon = [mbot_posizione_vera; mbot_posizione_vera2 ]; %to compute the correct distasnce between agents
    x_input  = [2, 5; 3, 6.6 ]; %vector of the information exchanged by agents
    x_input2 = [2, 5; 3, 6.6 ];
    n_agents= 5;
    % Inizializzo la classe
    wls_1 = wls_class(anchors, sigma_error,n_agents,x_input, mbot_posizione_vera, reference_positon); %n_agents è il numero di agenti, va specificato(n ancore+ nmbot)
    wls_2 = wls_class(anchors, sigma_error,n_agents,x_input2, mbot_posizione_vera2, reference_positon);
    % La funzione distance serve solo nella simulazione, poi sarà sostituita da

    % lettura UWB
    dist = wls_1.distance(x_input,mbot_posizione_vera); %inizializzo la prima
    z0 = wls_1.z_sensor(x_input, dist);
    [x_est, P] = wls_1.initialization(x_input,z0);

    dist2 = wls_2.distance(x_input2,mbot_posizione_vera2); %inizializzo la prima
    z02 = wls_2.z_sensor(x_input2, dist2);
    [x_est2, P2] = wls_2.initialization(x_input2,z02);


    sol(1,:, con)= [x_est(2),x_est(3), x_est2(2),x_est2(3)];
    x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
    x_input2 = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
    % Prime iterazioni con algoritmo per stimare posizioni iniziali
    for i=2:iter        
        dist = wls_1.distance(x_input,mbot_posizione_vera);
        [x_est, P] = wls_1.WLS3_distributed(x_est, P, x_input, dist);

        dist2 = wls_2.distance(x_input2,mbot_posizione_vera2);
        [x_est2, P2] = wls_2.WLS3_distributed(x_est2, P2, x_input2, dist2);  

        b1 = b1+1;
        k1 = k1+1;   
            if b1 == agg  
                x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg = random_n(min_a, max_b);  
                b1= 0; 
            end
             if k1 == agg2  
                x_input2 = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg2 = random_n(min_a, max_b); 
                k1= 0; 
             end


        sol(i,:,con) = [x_est(2),x_est(3), x_est2(2),x_est2(3)]; % salvo le soluzioni  
    end
    % Error calculation FIRST agent
    F1 = sol(:,1,con);
    F2 = sol(:,2,con);
    A1 = mbot_posizione_vera(1);
    A2 = mbot_posizione_vera(2);
    % https://it.mathworks.com/matlabcentral/answers/4064-rmse-root-mean-square-error
    E_sigma(con,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_sigma(con,2) = sqrt(mean((A2 - F2).^2));
    V_sigma(con,1) = var(F1);
    V_sigma(con,2) = var(F2);
    % SECOND agent
    F1 = sol(:,3,con);
    F2 = sol(:,4,con);
    A1 = mbot_posizione_vera2(1);
    A2 = mbot_posizione_vera2(2);

    E_sigma(con,3) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_sigma(con,4) = sqrt(mean((A2 - F2).^2));
    V_sigma(con,3) = var(F1);
    V_sigma(con,4) = var(F2);

end




%% Plotting the results
%Plot initial position estimation
    Mbot_1 = [x_est(2), x_est(3)];  % takes the last steps 


    fig = fig+1;
    figure(fig)
    tiledlayout(2,2)

    nexttile
    hold on
    plot(sol(:,1,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,1,40), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,1,70), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,1,100), 'linewidth',1.0 )
    yline(mbot_posizione_vera(1), 'd')
    str1 = sprintf('latency_1 = %i, %i',latency_vec(1,:));
    str2 = sprintf('latency_4 = %i, %i',latency_vec(40,:));
    str3 = sprintf('latency_7 = %i, %i',latency_vec(70,:));
    str4 = sprintf('latency{10} = %i, %i',latency_vec(100,:));
    legend(str1, str2, str3,str4)
    title('Estimation agent 1: x position')
    xlabel('iterations')
    ylabel('x position [m]')
    hold off

    nexttile
    hold on
    plot(sol(:,2,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,2,40), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,2,70), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,2,100), 'linewidth',1.0 )
    yline(mbot_posizione_vera(2), 'd')
    title('Estimation agent 1: y position')
    xlabel('iterations')
    ylabel('y position [m]')
    hold off

    nexttile
    hold on
    plot(sol(:,3,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,3,40), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,3,70), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,3,100), 'linewidth',1.0 )
    yline(mbot_posizione_vera2(1), 'd')
    str1 = sprintf('latency_1 = %i, %i',latency_vec(1,:));
    str2 = sprintf('latency_4 = %i, %i',latency_vec(40,:));
    str3 = sprintf('latency_7 = %i, %i',latency_vec(70,:));
    str4 = sprintf('latency{10} = %i, %i',latency_vec(100,:))
    legend(str1, str2, str3,str4)
    title('Estimation agent 2: x position')
    xlabel('iterations')
    ylabel('x position [m]')
    hold off

    nexttile
    hold on
    plot(sol(:,4,1), 'linewidth',1.0 )
    % plot(sol(:,1,2), 'linewidth',1.0 )
    % plot(sol(:,1,3), 'linewidth',1.0 )
    plot(sol(:,4,40), 'linewidth',1.0 )
    % plot(sol(:,1,5), 'linewidth',1.0 )
    % plot(sol(:,1,6), 'linewidth',1.0 )
    plot(sol(:,4,70), 'linewidth',1.0 )
    % plot(sol(:,1,8), 'linewidth',1.0 )
    % plot(sol(:,1,9), 'linewidth',1.0 )
    plot(sol(:,4,100), 'linewidth',1.0 )
    yline(mbot_posizione_vera2(2), 'd')
    title('Estimation agent 2: y position')
    xlabel('iterations')
    ylabel('y position [m]')
    hold off

    % Plotting RMSE
    fig = fig+1;
    figure(fig)
    tiledlayout(2,1)
    nexttile
    hold on
    plot(latency_vec(:,2),E_sigma(:,1), 'LineWidth',1.5)
    plot(latency_vec(:,2),E_sigma(:,2), 'LineWidth',1.5)
    plot(latency_vec(:,2),E_sigma(:,3), 'LineWidth',1.5)
    plot(latency_vec(:,2),E_sigma(:,4), 'LineWidth',1.5)
    legend('RMSE x1 axis', 'RMSE y1 axis', 'RMSE x2 axis', 'RMSE y2 axis')
    ylabel('RMSE [m]')
    xlabel('max possible latency [iterations]')
    hold off
    nexttile
    hold on
    plot(latency_vec(:,2), V_sigma(:,1), 'LineWidth',1.5)
    plot(latency_vec(:,2), V_sigma(:,2), 'LineWidth',1.5)
    plot(latency_vec(:,2), V_sigma(:,3), 'LineWidth',1.5)
    plot(latency_vec(:,2), V_sigma(:,4), 'LineWidth',1.5)
    legend('variance x1 axis', 'variance y1 axis', 'variance x2 axis', 'variance y2 axis')
    ylabel('Variance [m]')
    xlabel('max possible latency [iterations]')
    hold off
% Multiple simulation for having an histogram 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Faccio x volte la simulazione per avere una media del RMSE
n = 10;
iter = 100; %step to calculate the solution
E_multiple = zeros(n,4); % n = number of simulations
var_multiple = zeros(n,4);
sol_multiple = zeros(iter,4); % First element of the solution
n_agents = 5;
for e = 1:n
    % Re initialize the code to compute the position of moving Mbot with WLS
    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),... 
                   sqrt(0.22),...
                   sqrt(0.21),...
                   sqrt(0.21)]; %in meters
    %randomize the position

    mbot_vera_second = randi([0,5],1,2);
    mbot_vera_second2 = randi([0,5],1,2);
    x_input = [randi([0,7],1,2); randi([0,7],1,2)];
    x_input2 = [randi([0,7],1,2); randi([0,7],1,2)];
    reference_vera = [mbot_vera_second; mbot_vera_second2];

    min_a = 10; 
    max_b = 100;
    agg =  random_n(min_a, max_b); b1 = 0;
    agg2 = random_n(min_a, max_b); k1 =0;
    latency_value= [min_a, max_b];

    wls_mult = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second, reference_vera);
    wls_mult2 = wls_class(anchors, sigma_error,n_agents,x_input2, mbot_vera_second2, reference_vera);

    dist = wls_mult.distance(x_input,mbot_vera_second); %inizializzo la prima
    z0 = wls_mult.z_sensor(x_input, dist);
    [x_est, P] = wls_mult.initialization(x_input,z0);

    dist2 = wls_mult2.distance(x_input2,mbot_vera_second2); %inizializzo la prima
    z02 = wls_mult2.z_sensor(x_input2, dist2);
    [x_est2, P2] = wls_mult2.initialization(x_input2,z02);

    sol_multiple(1,:)= [x_est(2),x_est(3), x_est2(2),x_est2(3)];
    x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3)]; 
    x_input2 = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
    for l=2:iter
        dist = wls_mult.distance(x_input,mbot_vera_second);
        [x_est, P] = wls_mult.WLS3_distributed(x_est, P, x_input, dist);

        dist2 = wls_mult2.distance(x_input2,mbot_vera_second2);
        [x_est2, P2] = wls_mult2.WLS3_distributed(x_est2, P2, x_input2, dist2);

        b1 = b1+1;
        k1 = k1+1;   
            if b1 == agg  
                x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg = random_n(min_a, max_b);  
                b1= 0; 
            end
             if k1 == agg2  
                x_input2 = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg2 = random_n(min_a, max_b); 
                k1= 0; 
             end

        sol_multiple(l,:)= [x_est(2),x_est(3), x_est2(2),x_est2(3)];

    end
    %  Error calculation
    %FIRST
    F1 = sol_multiple(:,1);
    F2 = sol_multiple(:,2);
    A1 = mbot_vera_second(1);
    A2 = mbot_vera_second(2);
    E_multiple(e,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,2) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,1) = var(F1);
    var_multiple(e,2) = var(F2);
    % SECOND
    F1 = sol_multiple(:,3);
    F2 = sol_multiple(:,4);
    A1 = mbot_vera_second2(1);
    A2 = mbot_vera_second2(2);
    E_multiple(e,3) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,4) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,3) = var(F1);
    var_multiple(e,4) = var(F2);


end

 % Plot histogram
fig = fig+1;
figure(fig)
hold on
h =  histogram(E_multiple(:,1));
h2 = histogram(E_multiple(:,2));
h3 = histogram(E_multiple(:,3));
h4 = histogram(E_multiple(:,4));
legend('RMSE x1_{pos}', 'RMSE y1_{pos}', 'RMSE x2_{pos}', 'RMSE y2_{pos}')
ylabel('frequency')
xlabel('RMSE [m]')
title('RMSE of 1000 simulations with two agents and latency between 10 and 100 iterations')

Mean_RMSE_static_latency = mean(E_multiple)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%% Moving Mbot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To simulate the motion, a differential drive model is simulated, varying
% rpm of wheels
r2 = 6.4/2 *0.01; % radius of the wheel in cm
L2 = 11.2 *0.01;  % interasse in cm
% anchors = [A1;A2; A3];
% n_agents = 5;
% Faccio x volte la simulazione per avere una media del RMSE
n = 1000;
E_movement = zeros(n,4); % n = number of simulations
var_movement = zeros(n,4); 
for e = 1:n
    % initial conditions
    x_02 = randi([0,5],1);
    y_02 = randi([0,5],1);

    x_03 = randi([0,5],1);
    y_03 = randi([0,5],1);

    phi_02 = -3.14 + (3.14+3.14)*rand(1,1); %Initial angle in rad
    phi_03 = -3.14 + (3.14+3.14)*rand(1,1); %Initial angle in rad
    % This create the motion of the Mbot on a random trajectory
    Mbot_data = mbot_traj(r2, L2, x_02, y_02, phi_02); % output shape [time, x_pos, y_pos, phi, rpm_r_v, rpm_l_v]
    Mbot_data2 = mbot_traj(r2, L2, x_03, y_03, phi_03);
    % Simulation of WLS computation with moving Mbot
    % Now calculate at each time the estimate made by the wls recursive
    sol_movement = zeros(2000,4);

    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),...
                   sqrt(0.22),...
                   sqrt(0.21),...
                   sqrt(0.21)]; %in meters
    min_a_mov = 10; 
    max_b_mov = 100;
    agg = random_n(min_a_mov, max_b_mov);  b2 = 0;
    agg2 = random_n(min_a_mov, max_b_mov); k2 =0;
    latency_value= [min_a_mov, max_b_mov];
    % Re initialize the code to compute the position of moving Mbot with WLS
    x_input = [1,1.4; 2 ,3.4];
    x_input2 = [1,1.4; 2 ,3.4];
    mbot_vera_motion = [x_02, y_02; x_03, y_03 ];
    x_mbot = [Mbot_data(1,2), Mbot_data(1,3)]; % The initial position used to calculate distance in simulation    
    x_mbot2 = [Mbot_data2(1,2), Mbot_data2(1,3)]; % The initial position used to calculate distance in simulation

    wls_movement = wls_class(anchors, sigma_error,n_agents,x_input, x_mbot, mbot_vera_motion);
    wls_movement2 = wls_class(anchors, sigma_error,n_agents,x_input2, x_mbot2, mbot_vera_motion);
    
    dist = wls_movement.distance(x_input,x_mbot); %inizializzo la prima
    z0 = wls_movement.z_sensor(x_input, dist);
    [x_est, P] = wls_movement.initialization(x_input,z0);

    dist2 = wls_movement2.distance(x_input,x_mbot2); %inizializzo la seconda
    z02 = wls_movement2.z_sensor(x_input, dist2);
    [x_est2, P2] = wls_movement2.initialization(x_input,z02);
    
    sol_movement(1,:) = [x_est(2), x_est(3), x_est2(2), x_est2(3)]; % First element of the solution
    x_input = [x_est(2), x_est(3); x_est2(2), x_est2(3)];
    x_input2 = [x_est(2), x_est(3); x_est2(2), x_est2(3)];
    for l2=2:2000
        x_mbot = [Mbot_data(l2,2), Mbot_data(l2,3)];
        x_mbot2 = [Mbot_data2(l2,2), Mbot_data2(l2,3)];
        wls_movement.real_reference = [x_mbot; x_mbot2];
        wls_movement2.real_reference = [x_mbot; x_mbot2];

        if mod(l2,20)== 0 %ogni x misure reinizializzo, 20 sembra un minimo con questa dinamica di Movimento di Mbot
            dist = wls_movement.distance(x_input,x_mbot); %reinizializzo
            z0 = wls_movement.z_sensor(x_input, dist);
            [x_est, P] = wls_movement.initialization(x_input,z0);

            dist2 = wls_movement2.distance(x_input2,x_mbot2); %reinizializzo
            z02 = wls_movement2.z_sensor(x_input2, dist2);
            [x_est2, P2] = wls_movement2.initialization(x_input2,z02);

        end

        %at each iteration the position change
        dist = wls_movement.distance(x_input,x_mbot);
        [x_est, P] = wls_movement.WLS3_distributed(x_est, P, x_input, dist);

        dist2 = wls_movement2.distance(x_input2,x_mbot2);
        [x_est2, P2] = wls_movement2.WLS3_distributed(x_est2, P2, x_input2, dist2);


        %%%%%%%
        b2 = b2+1;
        k2 = k2+1;   
            if b2 == agg  
                x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg = random_n(min_a_mov, max_b_mov);  
                b2= 0; 
            end
             if k2 == agg2  
                x_input2 = [x_est(2),x_est(3); x_est2(2),x_est2(3)];
                agg2 = random_n(min_a_mov, max_b_mov); 
                k2= 0; 
             end

        % x_input = [x_est(2), x_est(3); x_est2(2), x_est2(3)];
        sol_movement(l2,:) = [x_est(2), x_est(3), x_est2(2), x_est2(3)];
    end
    % Error calculation
    % FIRST
    F1 = sol_movement(:,1);
    F2 = sol_movement(:,2);
    A1 = Mbot_data(:,2);
    A2 = Mbot_data(:,3);
    E_movement(e,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_movement(e,2) = sqrt(mean((A2 - F2).^2));
    var_movement(e,1) = var(A1-F1);
    var_movement(e,2) = var(A2-F2);

    % SECOND    
    F1 = sol_movement(:,3);
    F2 = sol_movement(:,4);
    A1 = Mbot_data2(:,2);
    A2 = Mbot_data2(:,3);
    E_movement(e,3) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_movement(e,4) = sqrt(mean((A2 - F2).^2));
    var_movement(e,3) = var(A1-F1);
    var_movement(e,4) = var(A2-F2);

end

%% variance hist
fig = fig+1;
figure(fig)
hold on
h5 = histogram(var_movement(:,1));
h6 = histogram(var_movement(:,2));
h7 = histogram(var_movement(:,3));
h8 = histogram(var_movement(:,4));
ylabel('frequency')
xlabel('variance')
title('variance of 1000 simulation with two moving agents and three anchors')
legend('var x1 axis', 'var y1 axis', 'var x2 axis', 'var y2 axis')
hold off

% rmse hist
fig = fig+1;
figure(fig)
hold on
h9 = histogram(E_movement(:,1));
h10 = histogram(E_movement(:,2));
h11 = histogram(E_movement(:,3));
h12 = histogram(E_movement(:,4));
ylabel('frequency')
xlabel('RMSE')
title('RMSE of 1000 simulation with two moving agents and three anchors')
legend('RMSE x1 axis', 'RMSE y1 axis', 'RMSE x2 axis', 'RMSE y2 axis')
hold off
Mean_RMSE_motion = mean(E_movement)

% Plot the figure

fig = fig+1;
figure(fig)
hold on
plot(sol_movement(:,1), sol_movement(:,2), '*')
plot(Mbot_data(:,2),Mbot_data(:,3), 'LineWidth',1.5)

plot(sol_movement(:,3), sol_movement(:,4), 'o')
plot(Mbot_data2(:,2),Mbot_data2(:,3), 'LineWidth',1.5)
% plot(sol_movement(:,5), sol_movement(:,6), 'og')
xlabel('x axis [m]')
ylabel('y axis [m]')
title('Mbot trajectory estimation')
legend( 'estimated position 1','ideal trajectory 1', 'estimated position 2','ideal trajectory 2')
% %% Animated plot  
% fig = fig+1;
% figure(fig)
% hold on
% h = animatedline('Color','b','LineWidth',0.75);
% g = animatedline('Color','r','LineStyle', ':');
% hh = animatedline('Color','b','LineWidth',0.75);
% gg = animatedline('Color','m','LineStyle', ':');
% A =  Mbot_data(:,2);   B = Mbot_data(:,3);
% AA = Mbot_data2(:,2); BB = Mbot_data2(:,3);
% C =  sol_movement(:,1);  D = sol_movement(:,2);
% CC = sol_movement(:,3); DD = sol_movement(:,4);
% for k = 1:length(A)
%     addpoints(h,A(k),B(k))
%     addpoints(g,C(k),D(k))
%     addpoints(hh,AA(k),BB(k))
%     addpoints(gg,CC(k),DD(k))
%      % xlim([min(C) max(C)]);
%      % ylim([min(D) max(D)]);
%     drawnow limitrate
%     pause(0.005)
% end
% hold off
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
%% Funzioni usate
function random_int = random_n(a,b)
    random_int = randi([a,b],1);
    %random_int = 1;
end