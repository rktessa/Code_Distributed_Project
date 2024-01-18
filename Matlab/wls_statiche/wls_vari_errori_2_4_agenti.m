clc;
clear all;
close all;

%% WLS code solution for Mbots
% Code with 2 Mbot and 3 Anchors
% Anchors
A1 = [0,0];
A2 = [9.0, 2.0];
A3 = [1, 9.0];
%A4 = [0, 8.0];
anchors = [A1;A2;A3];
fig = 0;
anchors = [A1];
n_agents = 5;
%% Multiple simulation for having an histogram 


% Faccio x volte la simulazione per avere una media del RMSE
n = 1000;
iter = 100;
E_multiple = zeros(n,8); % n = number of simulations
var_multiple = zeros(n,8);
sol_multiple = zeros(iter,8, n); % First element of the solution
ref_vera_vector = zeros(n,8);
for e = 1:n
    %% Re initialize the code to compute the position of moving Mbot with WLS
    sigma_error = [sqrt(0.21),...
                   sqrt(0.22),... 
                   sqrt(0.22),...
                   sqrt(0.21),...
                   sqrt(0.21),...
                   sqrt(0.21)]; %in meters
    %randomize the position
    
    mbot_vera_second = randi([0,9],1,2);
    mbot_vera_second2 = randi([0,9],1,2);
    mbot_vera_second3 = randi([0,9],1,2);
    mbot_vera_second4 = randi([0,9],1,2);

    x_input = [randi([0,15],1,2); randi([0,15],1,2);  randi([0,15],1,2) ;  randi([0,15],1,2)];
    reference_vera = [mbot_vera_second; mbot_vera_second2; mbot_vera_second3; mbot_vera_second3];
    ref_vera_vector(e,:) = [mbot_vera_second, mbot_vera_second2, mbot_vera_second3, mbot_vera_second4];

    wls_mult = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second, reference_vera);
    wls_mult2 = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second2, reference_vera);
    wls_mult3 = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second3, reference_vera);
    wls_mult4 = wls_class(anchors, sigma_error,n_agents,x_input, mbot_vera_second4, reference_vera);
    
    dist = wls_mult.distance(x_input,mbot_vera_second); %inizializzo la prima
    z0 = wls_mult.z_sensor(x_input, dist);
    [x_est, P] = wls_mult.initialization(x_input,z0);
    
    dist2 = wls_mult2.distance(x_input,mbot_vera_second2); %inizializzo la prima
    z02 = wls_mult2.z_sensor(x_input, dist2);
    [x_est2, P2] = wls_mult2.initialization(x_input,z02);

    dist3 = wls_mult3.distance(x_input,mbot_vera_second3); %inizializzo la prima
    z03 = wls_mult3.z_sensor(x_input, dist3);
    [x_est3, P3] = wls_mult3.initialization(x_input,z03);

    dist4 = wls_mult4.distance(x_input,mbot_vera_second4); %inizializzo la prima
    z04 = wls_mult4.z_sensor(x_input, dist4);
    [x_est4, P4] = wls_mult4.initialization(x_input,z04);
    
    sol_multiple(1,:,e)= [x_est(2),x_est(3), x_est2(2),x_est2(3), x_est3(2),x_est3(3), x_est4(2),x_est4(3)];
    x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3); x_est3(2),x_est3(3);  x_est4(2),x_est4(3)]; 

    for l=2:iter
        dist = wls_mult.distance(x_input,mbot_vera_second);
        [x_est, P] = wls_mult.WLS3_distributed(x_est, P, x_input, dist);
        
        dist2 = wls_mult2.distance(x_input,mbot_vera_second2);
        [x_est2, P2] = wls_mult2.WLS3_distributed(x_est2, P2, x_input, dist2);

        dist3 = wls_mult3.distance(x_input,mbot_vera_second3);
        [x_est3, P3] = wls_mult3.WLS3_distributed(x_est3, P3, x_input, dist3);

        dist4 = wls_mult4.distance(x_input,mbot_vera_second4);
        [x_est4, P4] = wls_mult4.WLS3_distributed(x_est4, P4, x_input, dist4);

        sol_multiple(l,:, e)= [x_est(2),x_est(3), x_est2(2),x_est2(3), x_est3(2),x_est3(3), x_est4(2),x_est4(3)];
        x_input = [x_est(2),x_est(3); x_est2(2),x_est2(3); x_est3(2),x_est3(3); x_est4(2),x_est4(3)];
    end
    %  Error calculation
    %FIRST
    F1 = sol_multiple(:,1, e);
    F2 = sol_multiple(:,2, e);
    A1 = mbot_vera_second(1);
    A2 = mbot_vera_second(2);
    E_multiple(e,1) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,2) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,1) = var(F1);
    var_multiple(e,2) = var(F2);
    % SECOND
    F1 = sol_multiple(:,3, e);
    F2 = sol_multiple(:,4, e);
    A1 = mbot_vera_second2(1);
    A2 = mbot_vera_second2(2);
    E_multiple(e,3) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,4) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,3) = var(F1);
    var_multiple(e,4) = var(F2);
    
    % THIRD
    F1 = sol_multiple(:,5, e);
    F2 = sol_multiple(:,6, e);
    A1 = mbot_vera_second3(1);
    A2 = mbot_vera_second3(2);
    E_multiple(e,5) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,6) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,5) = var(F1);
    var_multiple(e,6) = var(F2);
   

    % THIRD
    F1 = sol_multiple(:,7, e);
    F2 = sol_multiple(:,8, e);
    A1 = mbot_vera_second4(1);
    A2 = mbot_vera_second4(2);
    E_multiple(e,7) = sqrt(mean((A1 - F1).^2)); %Root Mean Square Error
    E_multiple(e,8) = sqrt(mean((A2 - F2).^2));
    var_multiple(e,7) = var(F1);
    var_multiple(e,8) = var(F2);
end

 %% Plot histogram
fig = fig+1;
figure(fig)
hold on
h =  histogram(E_multiple(:,1));
h2 = histogram(E_multiple(:,2));
h3 = histogram(E_multiple(:,3));
h4 = histogram(E_multiple(:,4));
h5 = histogram(E_multiple(:,5));
h6 = histogram(E_multiple(:,6));
h7 = histogram(E_multiple(:,7));
h8 = histogram(E_multiple(:,8));
legend('RMSE x1_{pos}', 'RMSE y1_{pos}', 'RMSE x2_{pos}',...
    'RMSE y2_{pos}', 'RMSE x3_{pos}', 'RMSE y3_{pos}',...
    'RMSE x4_{pos}', 'RMSE y4_{pos}');
ylabel('frequency')
xlabel('RMSE [m]')
title('RMSE of 1000 simulations with two agents and three anchors')

Mean_RMSE_static = mean(E_multiple)
%%

mean_riga = mean(E_multiple,2);
[min_err, min_index] = min(mean_riga)
[max_err, max_index] = max(mean_riga)
    
    fig = fig+1;
    figure(fig)
    tiledlayout(2,2)

    nexttile
    hold on
    plot(sol_multiple(:,1,min_index), 'linewidth',1.0 )
    plot(sol_multiple(:,2,min_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(min_index,1), 'd')
    yline(ref_vera_vector(min_index,2), 'd')
    plot(sol_multiple(:,1,max_index), 'linewidth',1.0 )
    plot(sol_multiple(:,2,max_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(max_index,1), 'd')
    yline(ref_vera_vector(max_index,2), 'd')
    legend('x', 'y');
    % str1 = sprintf('err_1 = %f',sigma_vec(1));
    % str2 = sprintf('err_4 = %f',sigma_vec(4));
    % str3 = sprintf('err_7 = %f',sigma_vec(7));
    % str4 = sprintf('err_{10} = %f',sigma_vec(10));
    % legend(str1, str2, str3,str4)
    
    title('Estimation agent 1: x-y position')
    xlabel('iterations')
    ylabel('position [m]')
    hold off

    nexttile
    hold on
    plot(sol_multiple(:,3,min_index), 'linewidth',1.0 )
    plot(sol_multiple(:,4,min_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(min_index,3), 'd')
    yline(ref_vera_vector(min_index,4), 'd')
     plot(sol_multiple(:,3,max_index), 'linewidth',1.0 )
    plot(sol_multiple(:,4,max_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(max_index,3), 'd')
    yline(ref_vera_vector(max_index,4), 'd')
    legend('x', 'y');
    title('Estimation agent 2: x-y position')
    xlabel('iterations')
    ylabel('position [m]')
    hold off

    nexttile
    hold on
    plot(sol_multiple(:,5,min_index), 'linewidth',1.0 )
    plot(sol_multiple(:,6,min_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(min_index,5), 'd')
    yline(ref_vera_vector(min_index,6), 'd')
     plot(sol_multiple(:,5,max_index), 'linewidth',1.0 )
    plot(sol_multiple(:,6,max_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(max_index,5), 'd')
    yline(ref_vera_vector(max_index,6), 'd')

    legend('x', 'y');
    title('Estimation agent 3: x-y position')
    xlabel('iterations')
    ylabel('position [m]')
    hold off

    nexttile
    hold on
    plot(sol_multiple(:,7,min_index), 'linewidth',1.0 )
    plot(sol_multiple(:,8,min_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(min_index,7), 'd');
    yline(ref_vera_vector(min_index,8), 'd');
     plot(sol_multiple(:,7,max_index), 'linewidth',1.0 )
    plot(sol_multiple(:,8,max_index), 'linewidth',1.0 )
    
    yline(ref_vera_vector(max_index,7), 'd')
    yline(ref_vera_vector(max_index,8), 'd')
    legend('x', 'y');
    title('Estimation agent 4: x-y position')
    xlabel('iterations')
    ylabel('position [m]')
    hold off
