clear all;

ptz_time=0;
target_order_ptz = [];
target_order = [];
X_obsv_pre=[];
risk_array=[];
% x=0; y=0; roll=0*pi/180; yaw=30*pi/180; V=10;
% x2=0; y2=100; roll2=8*pi/180; yaw2=-20*pi/180; V2=10;
actual_on = false;
sim_time = 500;
stare_time = 5;

avg_size = 10;
danger_area = 10;
distance_min = 100;
distance_max = 200;
position_angle_limit = 360;
roll_limit = 30;
yaw_limit = 360;
V_min = 20;
V_max = 10;
threat_number = 2;
T=0.1;
average_time = 5;
prediction_horizon = 50;
gimbal_position = [0 10];
n = 0;
safety_radius = 30;

safety_limit_angle = linspace(0,360,360);
safety_limit = [danger_area.*cos(safety_limit_angle); danger_area.*sin(safety_limit_angle)];
plot(0,0,'r*');
hold on;
plot(safety_limit(1,:), safety_limit(2,:), '.');
hold on;

% for i=1:threat_number
%     position_angle = rand*position_angle_limit*pi/180;
%     target(i,:) = [(rand*(distance_max-distance_min)+distance_min)*cos(position_angle) (rand*(distance_max-distance_min)+distance_min)*sin(position_angle) ...
%         (rand*roll_limit-roll_limit/2)*pi/180 rand*yaw_limit*pi/180 rand*(V_max-V_min)+V_min ...
%         randi(sim_time*0.8) (rand*roll_limit-roll_limit/2)*pi/180 randi(sim_time*0.8) (rand*roll_limit-roll_limit/2)*pi/180];
% end

target = [-50 20 -20*pi/180 0*pi/180 15 50 -20*pi/180 100 20*pi/180;
    -150 -150 20*pi/180 180*pi/180 15 50 -20*pi/180 100 20*pi/180];

[target(:,1), target(:,2), target(:,3)*180/pi, target(:,4)*180/pi, target(:,5), target(:,6), target(:,7)*180/pi, target(:,8), target(:,9)*180/pi ]
fprintf('      x         y        roll       yaw        V      time2     roll2     time3     roll3 \n');

X = [target(:,1), target(:,2), target(:,5).*cos(target(:,4)), target(:,5).*sin(target(:,4))];
X_obsv = zeros(threat_number,4);
Xdot_obsv = zeros(threat_number,4);
host_state = [-50 50];
host_control = [0 0];
final_state = [100; 50];
current_roll = target(:,3);

%% ------------------- Model ------------------- %%

host_X = [host_state(1); host_state(2); host_control(1); host_control(2)];
M = [1 0 1 0; 
    0 1 0 1;
    0 0 1 0;
    0 0 0 1];
N = [1 0;
    0 1;
    1 0;
    0 1];
Q = [1 0 0 0;
    0 1 0 0];
gamma = 1;

for i=1:prediction_horizon
    F(2*i-1:2*i,:) = Q * M^i;
    W(2*i-1:2*i,:) = final_state;
    Hmat = zeros(2,2);
    for j=1:i
        Hmat = Q * M^(j-1) * N + Hmat;
    end
    H(2*i-1:2*i,2*i-1:2*i) = Hmat;
end

u=zeros(prediction_horizon*2,1);

for k=1:sim_time
    
    % Change the roll angles (random roll, random timing)
    for i=1:threat_number
        if k==target(i,6)
            current_roll(i) = target(i,7);
        end
        if k==target(i,8)
            current_roll(i) = target(i,9);
        end
    end

    Omega = -sign(current_roll).*sqrt((1./cos(current_roll)).^2-1)*9.8./target(:,5);

    Xdot = [X(:,3) X(:,4) -Omega.*X(:,4) Omega.*X(:,3)];
    X = X + Xdot * T;
 
    %% ------------------- Actual Future Path ------------------- %%

    if actual_on == true

        X_actual = X;
        Xdot_actual = Xdot;

        for i=1:prediction_horizon
            X_actual = X_actual + Xdot_actual*T;
            Xdot_actual = [X_actual(:,3) X_actual(:,4) Xdot_actual(:,3) Xdot_actual(:,4)]; 
            for p=1:threat_number
                X_actual_hist{p}(i,:) = X_actual(p,1:2);
            end
        end

        for p=1:threat_number
            actual{p}=plot(X_actual_hist{p}(:,1), X_actual_hist{p}(:,2),'g.');
            hold on;
        end
        
    end
    
    %% ------------------- TSP ------------------- %%
    
    if k==1
        first_number = 0;
        target_order = tsp(threat_number, gimbal_position, X, k, first_number);
    end
    
    if rem(k,stare_time*(threat_number-1))==0
        first_number = target_order(end);
        gimbal_position = [X(target_order(end),1), X(target_order(end),2)];        
        target_order = tsp(threat_number, gimbal_position, X, k, first_number);  
    end
    
    %% ------------------- PTZ ------------------- %%
    n = n + 1;
    if k==1
        target_order(1) = [];
        target_order_ptz = target_order
    end
    
    % If risk array is completed, use it as the PTZ priority
    if isempty(risk_array) == false && n > stare_time * threat_number
        target_order_ptz = risk_array'
        n=1;
    end    
    
    if isempty(target_order_ptz) == false
        ptz_time = rem(ceil(n/stare_time),threat_number);
        if ptz_time == 0
            ptz_time = threat_number;
        end
        ptz_draw = plot(X(target_order_ptz(ptz_time),1), X(target_order_ptz(ptz_time),2),'ro', 'MarkerSize',10, 'LineWidth',2);
    end
    
    if k==1
        pre_ptz_time = ptz_time;
    end
    
    %% ------------------- Prediction ------------------- %%
    if isempty(target_order_ptz) == false
        
        % The data is received when the PTZ camera see the threat.
        X_obsv(target_order_ptz(ptz_time),1:2) = [X(target_order_ptz(ptz_time),1) X(target_order_ptz(ptz_time),2)];
        X_hist = [];

        % The velocity/acceleration is computed by averaging the position difference. 
        if isempty(X_obsv_pre) == false && pre_ptz_time == ptz_time % When the target is observed
            xdot_save(k) = X_obsv(target_order_ptz(ptz_time),1)-X_obsv_pre(target_order_ptz(ptz_time),1);
            ydot_save(k) = X_obsv(target_order_ptz(ptz_time),2)-X_obsv_pre(target_order_ptz(ptz_time),2);
        elseif isempty(X_obsv_pre) == false && pre_ptz_time ~= ptz_time % When the target is changed
            xdot_save(find(ismember(xdot_save,0)))=[];
            ydot_save(find(ismember(ydot_save,0)))=[];
            % Velocity
            X_obsv(pre_target_order_ptz(pre_ptz_time),3:4) = [mean(xdot_save) mean(ydot_save)]/T;
            Xdot_obsv(pre_target_order_ptz(pre_ptz_time),1:2) = [mean(xdot_save) mean(ydot_save)]/T;
            V_obsv = sqrt((mean(xdot_save)/T)^2 + (mean(ydot_save)/T)^2);
            % Acceleration            
            roll_obsv = current_roll(pre_target_order_ptz(pre_ptz_time));
            Omega_obsv = -sign(roll_obsv)*sqrt((1/cos(roll_obsv))^2-1)*9.8/V_obsv;
            Xdot_obsv(pre_target_order_ptz(pre_ptz_time),3:4) ...
                = [-Omega_obsv*mean(ydot_save)/T Omega_obsv*mean(xdot_save)/T];
            xdot_save = []; ydot_save = [];
        end
        
        % Propagate based on observation data
        X_obsv_pre = X_obsv;
        X_obsv = X_obsv + Xdot_obsv*T;
        Xdot_obsv = [X_obsv(:,3) X_obsv(:,4) Xdot_obsv(:,3) Xdot_obsv(:,4)];  
            
        % Predict based on observation data for prediction horizon
        X_pred = X_obsv;
        Xdot_pred = Xdot_obsv;

%         for i=1:prediction_horizon
%             for p=1:threat_number
%                 X_hist{p}(i,:) = [distance_max distance_max]
%             end
%         end
        
        for i=1:prediction_horizon
            X_pred = X_pred + Xdot_pred*T;
            Xdot_pred = [X_pred(:,3) X_pred(:,4) Xdot_pred(:,3) Xdot_pred(:,4)];
            for p=1:threat_number
                X_hist{p}(i,:) = X_pred(p,1:2);
            end
        end

        % Draw the prediction
        for p=1:threat_number
            pred{p}=plot(X_hist{p}(:,1), X_hist{p}(:,2),'r.');
            hold on;
        end
            
    end
    
        
    %% ------------------- Risk computation ------------------- %%
    
    % Check if PTZ see all the targets
    Xdot_pred(find(ismember(Xdot_pred(:,1),0)),:)=[];
    
    % Save all the distance of each threat using predicted path
    if size(Xdot_pred,1) == threat_number
        distance = [];
        for p=1:threat_number
            distance(p,:) = sqrt((X_hist{p}(:,1)-host_state(1)).^2 + (X_hist{p}(:,2)-host_state(2)).^2)';
        end

        % Compute relative velocity of each threat
        velocity = mean(distance(:,2:end) - distance(:,1:end-1),2)/T;

        % Compute risk of each threat
        risk = -velocity + 10000./distance(:,end);

        % Sort array in descend direction
        [risk_value, risk_array] = sort(risk,'descend');

        if min(velocity)<0
            risk_array';
        else
%             fprintf('Nothing is approaching\n');
        end

        
    end

    %% ------------------- Avoid ------------------- %%
    for i=1:prediction_horizon
        threat(2*i-1,:) = X_hist{1}(i,1);
        threat(2*i,:) = X_hist{1}(i,2);
%         threat(2*i-1,:) = 20;
%         threat(2*i,:) = i-1;    
    end
    
    H_tilde = H'*H+gamma;
    b_tilde = H'*(F*host_X - W);
    
    % Constraints (form of a*u < c)
    c = [inv(H)*(-safety_radius - threat + F*host_X), 0.5 * ones(size(u,1),1), 0.5 * ones(size(u,1),1)];
    % The sign of u
    u_sign = [-1 -1 1];
    
    % u means delta u
    stop = 100;
    while (stop>eps(1))
        d = -H_tilde*u - b_tilde;
        for j=1:size(u,1)
            if abs(d(j)) > eps(1)
                lamda_star = -(d(j)*H(j,j)*(H(j,j)*u(j) + F(j,1:4)*host_X - W(j)) + gamma*u(j))/(d(j)*H(j,j)*H(j,j)*d(j)+gamma*d(j));
                lamda_max_list = [(c(j,1)-u_sign(1)*u(j))/(u_sign(1)*d(j)), (c(j,2)-u_sign(2)*u(j))/(u_sign(2)*d(j)), (c(j,3)-u_sign(3)*u(j))/(u_sign(3)*d(j))];
                % Find only zero and positive value
                lamda_max_1 = lamda_max_list .* [(c(j,1)-u_sign(1)*u(j))/(u_sign(1)*d(j))>=0, (c(j,2)-u_sign(2)*u(j))/(u_sign(2)*d(j))>=0, (c(j,3)-u_sign(3)*u(j))/(u_sign(3)*d(j))>=0];
                % Change 0 and negative value to 10^(30)
                lamda_max_2 = 10^(30) .* [(c(j,1)-u_sign(1)*u(j))/(u_sign(1)*d(j))<0, (c(j,2)-u_sign(2)*u(j))/(u_sign(2)*d(j))<0, (c(j,3)-u_sign(3)*u(j))/(u_sign(3)*d(j))<0];
                lamda_max_list = lamda_max_1 + lamda_max_2;
                lamda_max = min(lamda_max_list);
                if lamda_max == 10^(30) || lamda_max < eps(1)
                    lamda(j,1) = 0;
                else
                    lamda(j,1) = min(lamda_max, lamda_star);
                end
            else
                d(j) = 0;
                lamda(j,1) = 0;
            end
            
        end
        
        u = u + lamda .* d;
        stop = sum(lamda.*d);
    end
    
    host = F*host_X + H*u;
    for i=1:prediction_horizon
        host_draw(i,1) = host(2*i-1);
        host_draw(i,2) = host(2*i);
    end
    host_plot=plot(host_draw(:,1), host_draw(:,2),'r');    
    
    
    %% ------------------- Draw ------------------- %%
    for i=1:threat_number
        a{i}=plot(X(i,1), X(i,2),'b.');
        hold on;
        t{i}=text(X(i,1)+1, X(i,2)+1,sprintf('%.f', i));
    end
    xlim([-distance_max distance_max])
    ylim([-distance_max distance_max])
    drawnow;

    for i=1:threat_number
        set(t{i},'Visible','off');
        set(a{i},'Visible','off');
        set(host_plot,'Visible','off');
        if actual_on == true
            set(actual{i},'Visible','off');
        end
    end
    
    if isempty(pred)==false
        for i=1:max(size(pred))
            set(pred{i},'Visible','off');                        
        end
    end
    
    if isempty(target_order_ptz) == false
        set(ptz_draw,'Visible','off');
    end
    
    pre_ptz_time = ptz_time;
    pre_target_order_ptz = target_order_ptz;
    
end
