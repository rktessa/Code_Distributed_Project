
%% Classe WLS per Mbots

classdef wls_class
    properties
        An
        sigma_e
        Cz
        n_agenti
        x_input % coordinate vere degli agenti servono solo per la simulazione
        mbot_posizione_vera 
        real_reference
    end
    
    methods
        function obj = wls_class(anchors, sigma_error, n_agenti, x_input, mbot_posizione_vera, real_reference) %definisco le cose che servono nella classe
                obj.An = anchors;
                obj.sigma_e = sigma_error;
                obj.n_agenti = n_agenti; %in base al numero di ancore + nume di agenti calcolo dimensione Cz
                obj.Cz = diag(obj.sigma_e(1:obj.n_agenti).^2);
                obj.x_input = x_input; %vettore coordinate vere
                obj.mbot_posizione_vera = mbot_posizione_vera; % serve per la funzione dist
                obj.real_reference = real_reference; 
        end
        
        function [x_est, P] = initialization(obj,x_input_est, z0)
            H = obj.H_matrix(x_input_est);
            P = pinv(H'*inv(obj.Cz)*H);
            x_est = P*H'*inv(obj.Cz)*z0;
            %x è il valore calcolato di posizione per Mbot, ma algh trova x^2+y^2, x,y
            %e mi servono solo gli ultimi due per x_input_est
                        
        end
        
        function [x_est, P] = WLS_distributed(obj,x_est, P, x_input_est)
            % Measurements of the sensor at each cycle 
            dist =  obj.distance(x_input_est, obj.mbot_posizione_vera);
            z = obj.z_sensor(x_input_est,dist);

            % Estimator
            H = obj.H_matrix(x_input_est);
            S = H*P*H' + obj.Cz;
            W = P*H'*pinv(S);
            x_est = x_est + W*(z - H*x_est);
            P = (eye(3) - W*H)*P;
     

        end
        % Measurements of the sensor at each cycle are from simulation    
        function [x_est, P] = WLS3_distributed(obj,x_est, P, x_input_est,z_dist)
            
            z = obj.z_sensor(x_input_est,z_dist);
            % Estimator
            H = obj.H_matrix(x_input_est);
            S = H*P*H' + obj.Cz;
            W = P*H'*pinv(S);
            x_est = x_est + W*(z - H*x_est);
            P = (eye(3) - W*H)*P;
     

        end
        
        
       
        % Alternative with changing Mbot position (for moving)
        function [x_est, P] = WLS2_distributed(obj,x_est, P, x_input_est, x_mbot)
            % Measurements of the sensor at each cycle 
            dist =  obj.distance(x_input_est,x_mbot);
            z = obj.z_sensor(x_input_est,dist);
            % Estimator
            H = obj.H_matrix(x_input_est);
            S = H*P*H' + obj.Cz;
            W = P*H'*pinv(S);
            x_est = x_est + W*(z - H*x_est);
            P = (eye(3) - W*H)*P;
        end
        
        function H = H_matrix(obj, x_input_est) %qui i valori stimati di vettore x!
            % Calculate H dimension
            row_An = size(obj.An); row_An = row_An(1);
            row_x = size(x_input_est);   row_x = row_x(1);
            % Initialize H 
            H = zeros((row_An+row_x),3);
            H(:,1) = 1; 
            % fill H
            for i=1:row_An
                H(i,2) = -2*obj.An(i,1);
                H(i,3) = -2*obj.An(i,2);
            end

            for j=row_An+1:(row_An+row_x)
                H(j,2) = -2*x_input_est(j-row_An,1);
                H(j,3) = -2*x_input_est(j-row_An,2);
            end
            
        end

        function z = z_sensor(obj,x_input_est,dist)
            % Calculate dimension
             row_An = size(obj.An); row_An = row_An(1);
             row_x = size(x_input_est);   row_x = row_x(1);
            % Initialize z
            z = zeros((row_An+row_x),1);

            % fill 
            for i=1:row_An
%                 disp(dist(i)^2);
                % z(i) = dist(i)^2 - norm(obj.An(i,:))^2; 
                    z(i) = dist(i)^2 - obj.An(i,1)^2 - obj.An(i,2)^2;
            end
            for i = (row_An+1): (row_An+row_x)
                % z(i) = dist(i)^2 - norm(x_input_est(i-row_An,:))^2;     
                  z(i) = dist(i)^2 - x_input_est(i-row_An,1)^2 - x_input_est(i-row_An,2)^2;   
            end
    
        end

        function dist =  distance(obj,x_input_est, x_mbot)%bisogna specificare rispetto a quale Mbot si calcola il vettore dist
            % x_input qui sono le dimensioni vere (note nella simulazione)!
            % Calculate  dimension
            row_An = size(obj.An); row_An = row_An(1);
            row_x = size(x_input_est);   row_x = row_x(1);
            % Initialize  
            dist = zeros((row_An+row_x),1);
            % fill
            for l=1:row_An
                dist(l) = norm(obj.An(l,:)-x_mbot) + randn(1,1)*obj.sigma_e(l);
                %dist(l) = norm(obj.An(l,:)-x_mbot) + randn(1,1)*obj.sigma_e(l);
            end
            for l=(row_An+1):(row_An+row_x)
                % dist(l) = norm(x_input_est(l-row_An,:)-x_mbot) + randn(1,1)*obj.sigma_e(l-row_An);
                dist(l) = norm(obj.real_reference(l-row_An,:)-x_mbot) + randn(1,1)*obj.sigma_e(l-row_An);
            end         
        end

        function dist =  distance_range(obj,x_input_est, x_mbot)%bisogna specificare rispetto a quale Mbot si calcola il vettore dist
            % x_input qui sono le dimensioni vere (note nella simulazione)!
            % Calculate  dimension
            row_An = size(obj.An); row_An = row_An(1);
            row_x = size(x_input_est);   row_x = row_x(1);
            % Initialize  
            dist = zeros((row_An+row_x),1);
            % fill
            for l=1:row_An
                meas = norm(obj.An(l,:)-x_mbot);
                dist(l) = obj.measure_err(meas)+ randn(1,1)*0.005;
                %dist(l) = norm(obj.An(l,:)-x_mbot) + randn(1,1)*obj.sigma_e(l);
                
            end
            for l=(row_An+1):(row_An+row_x)
                meas = norm(obj.real_reference(l-row_An,:)-x_mbot);
                dist(l) = obj.measure_err(meas)+ randn(1,1)*0.005;
            end         
        end

        % function that add error based on distance on the uwb 
        function y  = measure_err(obj,meas)
            error  = (0.017*meas-0.138); %error in meter
            y = meas + error;
        end

       
    end
    
end