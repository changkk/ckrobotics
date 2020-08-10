clear all;

% x=0; y=0; roll=0*pi/180; yaw=30*pi/180; V=10;
% x2=0; y2=100; roll2=8*pi/180; yaw2=-20*pi/180; V2=10;
actual_on = false;
draw = true;
sim_time = 1000;
stare_time = 10;
en=2.718281828;

avg_size = 10;
danger_area = 50;
distance_min = 80;
distance_max = 100;
position_angle_limit = 360;
roll_limit = 10;
yaw_limit = 360;
ep_max = 40;
V_min = 1;
V_max = 10; % velocity for 1s
number = 5; % Target numbers
orig_number = number;
T=0.1;
average_time = 5;
prediction_horizon = 100;
gimbal_control_limit = 40*pi/180; % speed for T
tracking_time_limit = 10;

sig_V = 0.1;
sig_W = 0.1;

safety_limit_angle = linspace(0,360,360);
safety_limit = [danger_area.*cos(safety_limit_angle); danger_area.*sin(safety_limit_angle)];
% plot(0,0,'r*');
% hold on;
plot(safety_limit(1,:), safety_limit(2,:), '.');
hold on;

    for i=1:number
        position_angle = rand*position_angle_limit*pi/180;
        target(i,:) = [(rand*(distance_max-distance_min)+distance_min)*cos(position_angle) (rand*(distance_max-distance_min)+distance_min)*sin(position_angle) ...
            (rand*roll_limit-roll_limit/2)*pi/180 rand*yaw_limit*pi/180 rand*(V_max-V_min)+V_min ...
            randi(sim_time*0.5) (rand*roll_limit-roll_limit/2)*pi/180 randi(sim_time*0.8) (rand*roll_limit-roll_limit/2)*pi/180 ...
            rand rand*yaw_limit*pi/180 rand*yaw_limit*pi/180];
    end
    
        %% ------------------- Good examples ------------------- %%

    % target=[    1.4569 -100.1708         0    5.2091    0.1627  216.0000         0  291.0000         0    0.5076 0 0
    %   128.9036   10.6331         0    0.4284    0.1581  255.0000         0  346.0000         0    0.2069 0 0
    %  -166.6450  -21.9114         0    4.4141    0.1956  178.0000         0   23.0000         0    0.7062 0 0
    %   -48.5864 -181.5418         0    5.9930    0.1074   83.0000         0  366.0000         0    0.8855 0 0
    %   106.8764  145.6047         0    1.4314    0.1321  332.0000         0  229.0000         0    0.5060 0 0]

    % target = [
    %  -100.3714 -156.1487         0    3.5711    0.1381  254.0000         0  164.0000         0    0.4684 0 0
    %  -191.0096   -2.5876         0    3.6073    0.1487  105.0000         0  352.0000         0    0.4409 0 0
    %   134.9240   77.7387         0    1.4645    0.1587  184.0000         0  265.0000         0    0.3472 0 0
    %    -4.5610  129.7847         0    2.2701    0.1742  283.0000         0    3.0000         0    0.9015 0 0
    %   -66.4940  117.9963         0    2.4401    0.1818  393.0000         0   34.0000         0    0.2361 0 0];


    % Figure 7
    % target = [   -100.0951  -10.0214         0  200.4865    1.5382   83.0000         0  145.0000         0    0.4139    3.0935    4.3652
    %    38.9197  -14.5595         0  343.5028    3   90.0000         0  113.0000         0    0.7111    3.9243    3.7109
    %    -7.6184  -35.0082         0   86.7258    7.1789  215.0000         0  293.0000         0    0.8367    0.8709    3.6958
    %   -55.0788   41.2443         0  200.7375    3.5961  113.0000         0   17.0000         0    0.1892    4.1916    3.6847
    %   -19.2642  -58.6716         0    6.9327    0.9303  244.0000         0   93.0000         0    0.1220    1.6867    1.6201];


    % Figure 8
%     target = [   -100.0951  -10.0214         0  200.4865    1.5382   83.0000         0  145.0000         0    0.4139    3.0935    4.3652
%        38.9197  -14.5595         0  183.5028    3   90.0000         0  113.0000         0    0.7111    3.9243    3.7109
%        -7.6184  -35.0082         0   86.7258    5.1789  215.0000         0  293.0000         0    0.8367    0.8709    3.6958
%       -55.0788   80.2443         0  200.7375    3.5961  113.0000         0   17.0000         0    0.1892    4.1916    3.6847
%       -50.2642  -70.6716         0    6.9327    2.9303  244.0000         0   93.0000         0    0.1220    1.6867    1.6201];

% target = [   74.4589   53.6657   -0.0724    1.3476    10  114.0000    0.0354  604.0000    0.0083    0.5535    3.9620    6.1918
%   -61.1591  -73.3535    0.0124    2.1075    9.5757  220.0000    0.0177  577.0000    0.0312    0.2128    0.5129    1.7246
%    61.3779  -66.0366   -0.0122    2    6.5738  329.0000   -0.0592  346.0000    0.0009    0.3753    3.0183    2.1515
%    14.8826  -92.8637   -0.0033    2    9.3818  259.0000    0.0703  175.0000    0.0651    0.0827    2.9242    0.1378
%    29.9280  -77.7845   -0.0556    3    10  150.0000    0.0468  401.0000    0.0715    0.0579    2.7442    3.5956];


    % target = [x y roll yaw V 50 -20*pi/180 100 20*pi/180;
    %     x2 y2 roll2 yaw2 V2 50 -20*pi/180 100 20*pi/180];

%     [target(:,1), target(:,2), target(:,3)*180/pi, target(:,4)*180/pi, target(:,5), target(:,6), target(:,7)*180/pi, target(:,8), target(:,9)*180/pi target(:,10) target(:,11) target(:,12)]
%     fprintf('      x         y        roll       yaw        V      time2     roll2     time3     roll3     risk 1      yaw2     yaw3 \n');

for sim_num = 1:2

    sim_num
    w_rv_list = [1, 0.5, 0];
    w_rv_for_main = w_rv_list(sim_num)

    X = [target(:,1), target(:,2), target(:,5).*cos(target(:,4)), target(:,5).*sin(target(:,4))];
    range = sqrt(target(:,1).^2 + target(:,2).^2);
    X_obsv = [target(:,1), target(:,2), zeros(number,2)]*100./range;
    sig_X = zeros(number,1);
    DE = zeros(number,1);
    
    gimbal_position = [0 10];
    gimbal_position_pid = [0 100];
    ptz_time=0;
    target_order_ptz = [];
    target_order = [];
    risk_array=[];
    n = 1;
    OnTheWay = true;
    p_error = 10000;
    Xdot_obsv = zeros(number,4);
    X_obsv_pre = zeros(number,4);
    current_state = [0 0];
    current_roll = target(:,3);
    prediction_start = false;
    pred=[];xdot_save=[];
    Omega_obsv=zeros(number,1);
    distance = [];
    target_number_covered = 0; control_sum=0; target_risk_covered=0;
    risk = zeros(number,1);
    scanning_over = false;
    obsv_error = 0.5;   % Observation error
    tracking_time = 0;

  %% ------------------- Simulation Start ------------------- %%
    for k=1:sim_time
    %     df = df + 0.01;
    %     df = min(1,df);
        idx = [];
        
%         % Find targets that inside security
%         if isempty(distance)==false
%             for i=1:number
%                 if distance(i,1) < 50 && distance(i,1) > 0 || distance(i,1) > 200
%                     idx(i) = 1;
%                 end
%             end
%         end
%         idx_arr = find(idx);
%         
%         % Delete a target
%         for i=1:size(idx_arr,2)
%                      X_obsv(idx_arr(i),:) = [];
%                     X(idx_arr(i),:) =[];
%                     X_actual(idx_arr(i),:) = [];
%                     X_obsv_pre(idx_arr(i),:) = [];
%                     X_pred(idx_arr(i),:) = [];
%                     Xdot(idx_arr(i),:) = [];
%                     Xdot_actual(idx_arr(i),:) = [];
%                     Xdot_obsv(idx_arr(i),:) = [];
%                     Xdot_pred(idx_arr(i),:) = [];
%                     Omega_obsv(idx_arr(i),:) = [];
% 
%                     number = number - 1;
%                     if scanning_over == false
%                         ptz_time = ptz_time - 1;
%                     end
%                     target(idx_arr(i),:) = [];
%                     current_roll(idx_arr(i),:) = [];
%                     sig_X(idx_arr(i)) = [];
%                     DE(idx_arr(i)) = [];
%                     DE_list(idx_arr(i),:) = [];
%                     distance(idx_arr(i),:) = [];
%                     prediction_error(idx_arr(i),:) = [];
%                     risk(idx_arr(i),:) = [];
%                     risk_print(idx_arr(i),:) = [];
%                     
%                     target_order_ptz(find(target_order_ptz == idx_arr(i)))=[];
%                     for j=1:number
%                         if target_order_ptz(j) > idx_arr(i)
%                             target_order_ptz(j) = target_order_ptz(j) - 1;
%                         end
%                         if pre_target_order_ptz(j) > idx_arr(i)
%                             pre_target_order_ptz(j) = pre_target_order_ptz(j) - 1;
%                         end
%                     end    
%             
%         end
        
        % If all targets are gone, break
        if(isempty(target)==true) 
            break;
        end
        
        % Change the roll angles (random roll, random timing)
        for i=1:number
            if k==target(i,6)
                current_roll(i) = target(i,7);
    %             X(i,3) = target(i,5).*cos(target(i,11));
    %             X(i,4) = target(i,5).*sin(target(i,11));
            end
            if k==target(i,8)
                current_roll(i) = target(i,9);
    %             X(i,3) = target(i,5).*cos(target(i,12));
    %             X(i,4) = target(i,5).*sin(target(i,12));
            end
        end

        Omega = -sign(current_roll).*sqrt((1./cos(current_roll)).^2-1)*(9.8)./target(:,5);

        Xdot = [X(:,3) X(:,4) -Omega.*X(:,4) Omega.*X(:,3)];
        X = X + Xdot * T;

        % Check if the initial scanning is over
        count = 0;
        if scanning_over == false
            for p=1:number
                if X_obsv(p,3) ~= 0
                    count = count +1;
                end
            end
        end

        if count == number
            scanning_over = true;
        end
        %% ------------------- TSP ------------------- %%

        if k==1
            first_number = 0;
            w_rv = 0;
            target_order = tsp_ortho_new(number, gimbal_position, X, k, first_number, risk, w_rv)
        end
    %     
    %     if rem(k,stare_time*(number-1))==0
    %         first_number = target_order(end);
    %         gimbal_position = [X(target_order(end),1), X(target_order(end),2)];       
    %         risk = 
    %         target_order = tsp_ortho(number, gimbal_position, X, k, first_number, risk);  
    %     end

        %% ------------------- PTZ ------------------- %%
        if k==1
            %target_order(1) = [];
            target_order_ptz = target_order;
        end

        % If risk array is completed, use it as the PTZ priority
        if n > stare_time * 1 && scanning_over == true
            first_number = target_order_ptz(end);
            risk';
            gimbal_position = [X(first_number,1), X(first_number,2)];
            future_X = [];
            for p=1:number
                future_X(p,1) = X_hist{p}(prediction_horizon/2,1);
                future_X(p,2) = X_hist{p}(prediction_horizon/2,2);
            end
            target_order_ptz = tsp_ortho_new(number, gimbal_position, future_X, k, first_number, risk, w_rv_for_main);
            n=1;
        end    

        if isempty(target_order_ptz) == false && scanning_over == false
            ptz_time = rem(ceil(n/stare_time),orig_number);
            if ptz_time == 0
                ptz_time = number;
            end
        else
            ptz_time = 1;
        end
        ptz_idx = target_order_ptz(ptz_time);

        if k~=1 && pre_ptz_idx ~= ptz_idx
            OnTheWay = true;
            tracking_time = 0;
        end

        %% ------------------- Draw gimbal ------------------- %%
        if isempty(target_order_ptz) == false && OnTheWay == true && draw == true
            if X_obsv(target_order_ptz(ptz_time),3) == 0
                range = sqrt(X(target_order_ptz(ptz_time),1).^2 + X(target_order_ptz(ptz_time),2).^2);
                ptz_draw = plot(X(target_order_ptz(ptz_time),1)*100/range, X(target_order_ptz(ptz_time),2)*100/range,'ro', 'MarkerSize',10, 'LineWidth',2);
                hold on;         
            else
                ptz_draw = plot(X_obsv(target_order_ptz(ptz_time),1), X_obsv(target_order_ptz(ptz_time),2),'ro', 'MarkerSize',10, 'LineWidth',2);
                hold on;
            end
            ptz_line_draw=plot([0, gimbal_position_pid(1)],[0, gimbal_position_pid(2)],'b','MarkerSize',10, 'LineWidth',2);
        end

        if isempty(target_order_ptz) == false && OnTheWay == false
            if draw == true
                ptz_draw = plot(X(target_order_ptz(ptz_time),1), X(target_order_ptz(ptz_time),2),'ro', 'MarkerSize',10, 'LineWidth',2);
                hold on;         
                ptz_line_draw =plot([0, X(target_order_ptz(ptz_time),1)],[0, X(target_order_ptz(ptz_time),2)],'b','MarkerSize',10, 'LineWidth',2);
            end
            n = n + 1;
        end

        if k==1
            pre_ptz_idx = ptz_idx;
            pre_ptz_time = ptz_time;
        end

        %% ------------------- Prediction ------------------- %%
        % When the target is first observed
        if isempty(target_order_ptz) == false && OnTheWay == false % When the target is being observed
            % The data is received when the PTZ camera see the threat.
            X_obsv(target_order_ptz(ptz_time),1:2) = [normrnd(X(target_order_ptz(ptz_time),1),obsv_error) normrnd(X(target_order_ptz(ptz_time),2),obsv_error)];
            K_X = sig_X(target_order_ptz(ptz_time), 1)/(sig_X(target_order_ptz(ptz_time), 1) + sig_V);
            sig_X(target_order_ptz(ptz_time), 1) = (1-K_X)*sig_X(target_order_ptz(ptz_time), 1);
            X_hist = [];
            % When the target is first observed, save the previous data for the speed estimation
            if X_obsv_pre(target_order_ptz(ptz_time),3) == 0 && isempty(xdot_save) == true 
                X_obsv_pre = X_obsv;
                % The velocity/acceleration is computed by averaging the position difference. 
                xdot_save(k) = X_obsv(target_order_ptz(ptz_time),1)-X_obsv_pre(target_order_ptz(ptz_time),1);
                ydot_save(k) = X_obsv(target_order_ptz(ptz_time),2)-X_obsv_pre(target_order_ptz(ptz_time),2);
                % and start prediction
                prediction_start = true;
            else
                % The velocity/acceleration is computed by averaging the position difference. 
                xdot_save(k) = X_obsv(target_order_ptz(ptz_time),1)-X_obsv_pre(target_order_ptz(ptz_time),1);
                ydot_save(k) = X_obsv(target_order_ptz(ptz_time),2)-X_obsv_pre(target_order_ptz(ptz_time),2);
            end   
        elseif isempty(X_obsv_pre) == false && pre_ptz_idx ~= ptz_idx && isempty(xdot_save) == false % When the target is changed
            target_number_covered = target_number_covered + 1;
            % Delete zeros to get only position difference
            xdot_save(find(ismember(xdot_save,0)))=[];
            ydot_save(find(ismember(ydot_save,0)))=[];
            % Delete errorneous velocity (first one has a high error
            % because of the previous estimates
            if size(xdot_save,2) > 0
                xdot_save(1) =[];
            else
                xdot_save = 0;
            end
            if size(ydot_save,2) > 0
                ydot_save(1) =[];
            else
                ydot_save = 0;
            end
            
            if pre_target_order_ptz(pre_ptz_time) <= size(current_roll,1) % Update this only when the previous target is not deleted.
                % Velocity
                X_obsv(pre_target_order_ptz(pre_ptz_time),3:4) = [mean(xdot_save) mean(ydot_save)]/T;
                Xdot_obsv(pre_target_order_ptz(pre_ptz_time),1:2) = [mean(xdot_save) mean(ydot_save)]/T;
                V_obsv = sqrt((mean(xdot_save)/T)^2 + (mean(ydot_save)/T)^2);
                % Acceleration            
                roll_obsv = current_roll(pre_target_order_ptz(pre_ptz_time));
                Omega_obsv(pre_target_order_ptz(pre_ptz_time),1) = -sign(roll_obsv)*sqrt((1/cos(roll_obsv))^2-1)*9.8/V_obsv;
            end
            xdot_save = []; ydot_save = [];
        end


        if prediction_start == true

            % Propagate based on observation data
            X_obsv_pre = X_obsv;
            X_obsv = X_obsv + Xdot_obsv*T;
            Xdot_obsv = [X_obsv(:,3) X_obsv(:,4) -Omega_obsv.*X_obsv(:,4) Omega_obsv.*X_obsv(:,3)];
            sig_X = sig_X + sig_W;
            DE = 0.5 * log((2*pi*en)^2.*sig_X);

            % Predict based on observation data for prediction horizon
            X_pred = X_obsv;
            Xdot_pred = Xdot_obsv;

            for i=1:prediction_horizon
                X_pred = X_pred + Xdot_pred*T;
                Xdot_pred = [X_pred(:,3) X_pred(:,4) -Omega_obsv.*X_pred(:,4) Omega_obsv.*X_pred(:,3)];
                for p=1:number
                    X_hist{p}(i,:) = X_pred(p,1:2);
                end
            end

            % Draw the prediction
            if draw == true
                for p=1:number
                    if X_pred(p,3)~=0
                        pred{p}=plot(X_hist{p}(:,1), X_hist{p}(:,2),'r.');
                        hold on;
                    end
                end
            end

        end

        %% ------------------- PID controller ------------------- %%
        if isempty(target_order_ptz) == false && OnTheWay == true

            Kp = 0.1; Kd = 0.1; % PID Controller parametrs
            state = gimbal_position_pid;
            angle_state = atan2(-state(1), state(2));
            goal = [X(target_order_ptz(ptz_time),1), X(target_order_ptz(ptz_time),2)];
            angle_goal = atan2(-goal(1), goal(2));
            p_error_prev = 0;
            p_error = angle_goal - angle_state;
            if abs(angle_goal-angle_state)>pi && angle_goal-angle_state < 0
                p_error = angle_goal - angle_state + 2*pi;
            elseif abs(angle_goal-angle_state)>pi && angle_goal-angle_state > 0
                p_error = angle_goal - angle_state - 2*pi;
            end
            d_error = p_error - p_error_prev;
            control = Kp*p_error + Kd*d_error;
            control = min(gimbal_control_limit, control);
            angle_state = angle_state + control;
            gimbal_position_pid = [-100*sin(angle_state), 100*cos(angle_state)];
            control_sum = control_sum + abs(control);
            tracking_time = tracking_time + 1;
            if norm(p_error)<5*pi/180 || tracking_time > tracking_time_limit
                OnTheWay = false;
            end
        end

        %% ------------------- Actual Future Path ------------------- %%

        if prediction_start == true

            X_actual = X;
            Xdot_actual = Xdot;

            for i=1:prediction_horizon
                X_actual = X_actual + Xdot_actual*T;
                Xdot_actual = [X_actual(:,3) X_actual(:,4) -Omega.*X_actual(:,4) Omega.*X_actual(:,3)]; 
                for p=1:number
                    X_actual_hist{p}(i,:) = X_actual(p,1:2);
                end
            end

            for p=1:number
                if X_pred(p,3)~=0 && actual_on == true
                    actual{p}=plot(X_actual_hist{p}(:,1), X_actual_hist{p}(:,2),'g.');
                    hold on;
                end
            end

        end

        %% ------------------- Risk ------------------- %%

        % Save all the distance of each threat using predicted path
        if prediction_start == true
            for p=1:number
                distance(p,:) = sqrt((X_hist{p}(:,1)-current_state(1)).^2 + (X_hist{p}(:,2)-current_state(2)).^2)';
            end

            % Compute relative velocity of each threat
            velocity_cur = mean(distance(:,2:prediction_horizon/2) - distance(:,1:(prediction_horizon/2-1)),2)/T;
            velocity_future = mean(distance(:,prediction_horizon/2:end) - distance(:,(prediction_horizon/2-1):end-1),2)/T;

            for p=1:number
                prediction_error(p,:) =sqrt((X_actual_hist{p}(:,1)-X_hist{p}(:,1)).^2 + (X_actual_hist{p}(:,2)-X_hist{p}(:,2)).^2)';
            end

            ep = mean(prediction_error,2);
            if scanning_over == true
                ep_list(:,k) = mean(ep);
            end
            % Compute risk of each threat
            PPA = min(0,ep/ep_max);
            ASV = max(0,-velocity_cur/V_max);
            ASF = max(0,-velocity_future/V_max.*PPA);
            R = max(0, 1-distance(:,1)/distance_max);

            risk = ASV * 0.2 + ASF.* PPA * 0.3 + R * 0.5 + DE;
            risk_print = ASV * 0.2 + ASF.* PPA * 0.3 + R * 0.5;           
        end


        %% ------------------- Draw ------------------- %%
        
        if draw == true
            for i=1:number
                if actual_on == true
                    a{i}=plot(X(i,1), X(i,2),'b.');
                    hold on;
                end
                
                if X_obsv(i,3) == 0
                    range = sqrt(X(i,1).^2 + X(i,2).^2);
                    est{i}=plot(X(i,1)*100/range, X(i,2)*100/range,'bo','MarkerSize', 8);
                    t1{i}=text(X(i,1)*100/range+8, X(i,2)*100/range+10,sprintf('%.f',i),'FontSize', 11);
                else
                    est{i}=plot(X_obsv(i,1), X_obsv(i,2),'bo','MarkerSize', 8);
                    t1{i}=text(X_obsv(i,1)+8, X_obsv(i,2)+3,sprintf('%.f',i),'FontSize', 11);
                    t2{i}=text(X_obsv(i,1)+18, X_obsv(i,2)+3,sprintf(': %.2f',risk_print(i)),'FontSize', 11);
                end
            end

            if isempty(target_order_ptz) == false 
                t3=text(-190, 180, sprintf('Target schedule'), 'FontSize', 15);
                t4=text(-55, 180, sprintf('-%.d',target_order_ptz), 'FontSize', 15);
            end

            xlim([-distance_max-100 distance_max+100])
            ylim([-distance_max-100 distance_max+100])
            xlabel('\fontsize{18} x (m)');
            ylabel('\fontsize{18} y (m)');

            drawnow;

            for i=1:number
                set(t1{i},'Visible','off');
                if X_obsv_pre(i,3) ~= 0
                    set(t2{i},'Visible','off');
                end
                if actual_on == true
                    set(a{i},'Visible','off');
                end
                set(est{i},'Visible','off');
                if actual_on == true && X_obsv(i,3)~=0
                    set(actual{i},'Visible','off');
                end
            end

            set(t3,'Visible','off');
            set(t4,'Visible','off');

            if isempty(pred)==false
                for i=1:max(size(pred))
                    set(pred{i},'Visible','off');                        
                end
            end

            if isempty(target_order_ptz) == false
                set(ptz_draw,'Visible','off');
                set(ptz_line_draw,'Visible','off');
            end
        end
        pre_ptz_idx = ptz_idx;
        pre_ptz_time = ptz_time;
        pre_target_order_ptz = target_order_ptz;
        target_number_covered_list(:,k) = [k; target_number_covered];
        DE_list2(:,k) = [k; sum(DE)];
    end
    subplot(2,2,1)
    plot(target_number_covered_list(1,:),target_number_covered_list(2,:));
    hold on;
    subplot(2,2,2)
    plot(DE_list2(1,:),DE_list2(2,:));
    mean(DE_list2,2)
    hold on;
    subplot(2,2,3)
    plot(ep_list);
    hold on;
    target_number_covered_list = [];
    DE_list2 = [];
    ep_list = [];
end