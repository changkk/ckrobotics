clear all;



% x=0; y=0; roll=0*pi/180; yaw=30*pi/180; V=10;
% x2=0; y2=100; roll2=8*pi/180; yaw2=-20*pi/180; V2=10;
actual_on = true;
draw = true;
sim_time = 3000;
stare_time = 15; % stare time for 1 T
sig_V = 25;
sig_W = 0.1;
obsv_error = 0;   % Observation error
%w_rv_list = [1, 0.5, 0.1];
w_rv_list = linspace(0,1,5);

en=2.718281828;
avg_size = 10;
danger_area = 10;
distance_min = 100;
distance_max = 300;
position_angle_limit = 360;
roll_limit = 10;
yaw_limit = 360;
ep_max = 40;
V_min = 10;
V_max = 20; % velocity for 1s
R_max = 500;
number = 10; % Target numbers
T=0.1;
average_time = 5;
prediction_horizon = 100;
gimbal_control_limit = 40*pi/180; % speed for T

n = 1;
OnTheWay = true;

safety_limit_angle = linspace(0,360,360);
safety_limit = [danger_area.*cos(safety_limit_angle); danger_area.*sin(safety_limit_angle)];
% plot(0,0,'r*');
% hold on;
%plot(safety_limit(1,:), safety_limit(2,:), '.');
%hold on;

if(draw==false) 
    f1 = figure(1);
    f2 = figure(2);
    f3 = figure(3);
end

for sim_num = 1:1
    
    sim_num
    
    for i=1:number
        position_angle = rand*position_angle_limit*pi/180;
        target(i,:) = [(rand*(distance_max-distance_min)+distance_min)*cos(position_angle) (rand*(distance_max-distance_min)+distance_min)*sin(position_angle) ...
            (rand*roll_limit-roll_limit/2)*pi/180 rand*yaw_limit*pi/180 rand*(V_max-V_min)+V_min ...
            randi(sim_time*0.5)*0 (rand*roll_limit-roll_limit/2)*pi/180 randi(sim_time*0.8)*0 (rand*roll_limit-roll_limit/2)*pi/180 ...
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
    % 
    % target = [  -68.6097   56.3879   -0.0366   0   12.6528         0    0.0842         0   -0.0272    0.5841    0.6771    5.6945
    %    70.0963  -58.4684    0.0165    3   14.2526         0   -0.0591         0   -0.0135    0.0942    3.7606    2.9589
    %   -31.3125  -87.4720   -0.0814    0.4323   13.1960         0    0.0270         0    0.0558    0.7184    6.0862    3.3385
    %   -37.3453   82.1295    0.0487    -0.5   10.9082         0   -0.0604         0   -0.0105    0.5271    2.8741    5.5001
    %   -98.2371  -10.4981    0.0799    1   16.7612         0    0.0300         0   -0.0754    0.2548    1.4077    4.1961];

        % target = [x y roll yaw V 50 -20*pi/180 100 20*pi/180;
        %     x2 y2 roll2 yaw2 V2 50 -20*pi/180 100 20*pi/180];
% 
%         [target(:,1), target(:,2), target(:,3)*180/pi, target(:,4)*180/pi, target(:,5), target(:,6), target(:,7)*180/pi, target(:,8), target(:,9)*180/pi target(:,10) target(:,11) target(:,12)]
%         fprintf('      x         y        roll       yaw        V      time2     roll2     time3     roll3     risk 1      yaw2     yaw3 \n');


    for wrv_num = 1:5

        w_rv_for_main = w_rv_list(wrv_num)

        X = [target(:,1), target(:,2), target(:,5).*cos(target(:,4)), target(:,5).*sin(target(:,4))];
        range = sqrt(target(:,1).^2 + target(:,2).^2);
        X_obsv = [target(:,1), target(:,2), zeros(number,2)]*100./range;
        sig_X = zeros(number,1);
        DE = zeros(number,1);
        ptz_time=0;
        Xdot_obsv = zeros(number,4);
        X_obsv_pre = zeros(number,4);
        current_state = [0 0];
        current_roll = target(:,3);
        prediction_start = false;
        pred=[];xdot_save=[];
        gimbal_position = [0 10];
        gimbal_position_pid = [0 100];
        target_order_ptz = [];
        target_order = [];
        risk_array=[];
        Omega_obsv=zeros(number,1);
        target_number_covered = 0; control_sum=0; target_risk_covered=0;
        risk = target(:,10);
        p_error = 10000;
        scanning_over = false;

          %% ------------------- Simulation Start ------------------- %%
        for k=1:sim_time
        %     df = df + 0.01;
        %     df = min(1,df);

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

            Omega = -sign(current_roll).*sqrt((1./cos(current_roll)).^2-1)*9.8./target(:,5);

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
                target_order = tsp_ortho_new(number, gimbal_position, X, k, first_number, risk, w_rv);
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
            if n > stare_time * number && scanning_over == true
                first_number = target_order_ptz(end);
                gimbal_position = [X(first_number,1), X(first_number,2)];
                for p=1:number
                    future_X(p,1) = X_hist{p}(prediction_horizon/2,1);
                    future_X(p,2) = X_hist{p}(prediction_horizon/2,2);
                end
                target_order_ptz = tsp_ortho_new(number, gimbal_position, future_X, k, first_number, risk, w_rv_for_main);
                n=1;
                r=1; risk_combine = 0;
                while r < size(risk,1)/2+1
                    risk_combine = risk_combine + risk(target_order_ptz(r));
                    r = r + 1;
                end
            end    

            if isempty(target_order_ptz) == false
                ptz_time = rem(ceil(n/stare_time),number);
                if ptz_time == 0
                    ptz_time = number;
                end
            end
            ptz_idx = target_order_ptz(ptz_time);

            if k~=1 && pre_ptz_idx ~= ptz_idx
                OnTheWay = true;
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
            if isempty(target_order_ptz) == false && OnTheWay == false 
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
                target_risk_covered = target_risk_covered + target(target_order_ptz(pre_ptz_time),10);
                xdot_save(find(ismember(xdot_save,0)))=[];
                ydot_save(find(ismember(ydot_save,0)))=[];
                % Delete errorneous velocity (first one has a high error
                % because of the previous estimates
                if size(xdot_save,2) > 1
                    xdot_save(1) =[];
                else
                    xdot_save = 0;
                end
                if size(ydot_save,2) > 1
                    ydot_save(1) =[];
                else
                    ydot_save = 0;
                end
                % Velocity
                X_obsv(pre_target_order_ptz(pre_ptz_time),3:4) = [mean(xdot_save) mean(ydot_save)]/T;
                Xdot_obsv(pre_target_order_ptz(pre_ptz_time),1:2) = [mean(xdot_save) mean(ydot_save)]/T;
                V_obsv = sqrt((mean(xdot_save)/T)^2 + (mean(ydot_save)/T)^2);
                % Acceleration            
                roll_obsv = current_roll(pre_target_order_ptz(pre_ptz_time));
                if V_obsv ~= 0
                    Omega_obsv(pre_target_order_ptz(pre_ptz_time),1) = -sign(roll_obsv)*sqrt((1/cos(roll_obsv))^2-1)*9.8/V_obsv;
                else
                    Omega_obsv(pre_target_order_ptz(pre_ptz_time),1) = 0;
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
                if norm(p_error)<5*pi/180
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
                    if X_pred(p,3)~=0 && actual_on == true && draw == true
                         actual{p}=plot(X_actual_hist{p}(:,1), X_actual_hist{p}(:,2),'g.');
                         hold on;
                    end
                end

            end

            %% ------------------- Risk ------------------- %%

            % Save all the distance of each threat using predicted path
            if prediction_start == true
                distance = [];
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
                    ep_list(:,k) = ep;
                end

                % Compute risk of each threat
                PPA = min(1,ep/ep_max);
                ASV = max(0,-velocity_cur/V_max);
                ASF = max(0,-velocity_future/V_max.*PPA);
                R = max(0, 1-distance(:,1)/R_max);

                risk = ASV * 0.3 + ASF.* PPA * 0.3 + R * 0.4;

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
                        t2{i}=text(X_obsv(i,1)+18, X_obsv(i,2)+3,sprintf(': %.2f',risk(i)),'FontSize', 11);
                    end
                end

                if isempty(target_order_ptz) == false 
                    t3=text(-R_max-190, R_max+180, sprintf('Target schedule'), 'FontSize', 15);
                    t4=text(-R_max-55, R_max+180, sprintf('-%.d',target_order_ptz), 'FontSize', 15);
                end

                xlim([-R_max-100 R_max+100])
                ylim([-R_max-100 R_max+100])
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
            %target_number_covered_list(:,k) = [k; target_number_covered];
            DE_list(:,k) = DE;

        end

        if draw == false

    %         figure(f1);
    %         plot(target_number_covered_list(1,:)/10,target_number_covered_list(2,:));
    %         hold on;
    %         grid minor;
    %         grid on;
    %         xlabel('\fontsize{16} Time (sec)'); ylabel('\fontsize{16} Number of targets observed');
    %             
    %         figure(f2);
    %         for i=1:5
    %             subplot(5,1,i)
    %             plot(target_number_covered_list(1,:)/10,ep_list(i,:));
    %             grid minor;
    %             grid on;
    %             hold on;
    %         end
    %         han=axes(f2,'visible','off'); 
    %         han.Title.Visible='on';
    %         han.XLabel.Visible='on';
    %         han.YLabel.Visible='on';
    %         ylabel(han,'\fontsize{16} e_p');
    %         xlabel(han,'\fontsize{16} Time (sec)');
    %         
    %         figure(f3);
    %         grid minor;
    %         grid on;
    % 
    %         for i=1:5
    %             subplot(5,1,i)
    %             ax{i}=plot(target_number_covered_list(1,:)/10,DE_list(i,:));
    %             grid minor;
    %             grid on;
    %             hold on;
    %         end
    %         han2=axes(f3,'visible','off'); 
    %         han2.Title.Visible='on';
    %         han2.XLabel.Visible='on';
    %         han2.YLabel.Visible='on';
    %         ylabel(han2,'\fontsize{16} Differential entrophy');
    %         xlabel(han2,'\fontsize{16} Time (sec)');
        end
        
        mean_ep(sim_num, wrv_num) = mean(mean(ep_list,2));
        mean_DE(sim_num, wrv_num) = mean(mean(DE_list,2));
        target_number_covered_list(sim_num, wrv_num) = target_number_covered;
        risk_combine_list(sim_num,wrv_num) = risk_combine;
        
    %     target_number_covered_list = [];
    %     DE_list = [];
    %     ep_list = [];
    end
    
    if (sim_num == 20)
        
            figure(f1);
            bar(target_number_covered_list);
            ylabel('\fontsize{16} Number of targets');
            xlabel('\fontsize{16} Time (sec)');
            grid minor;
            grid on;
            
            figure(f2);
            bar(mean_ep);
            ylabel('\fontsize{16} e_p');
            xlabel('\fontsize{16} Time (sec)');
            grid minor;
            grid on;
            
            figure(f3);
            bar(mean_DE);
            ylabel('\fontsize{16} DE');
            xlabel('\fontsize{16} Time (sec)');
            grid minor;
            grid on;
    end
    
    
end

