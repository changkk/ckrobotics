
x = [-161.9057   41.0388   26.2742   41.4972   36.1100    1.1327         0         0         0  -62.4461   39.9667   -0.4304];
gamma = [     0     0
     0     0
     1     1];
fun = 0;

for i = 1:target_num
    % Find the drones of each target
    temp = find(gamma(:,i));
    % x index of two targets e.g.) x1 = x(1), y1 = x(2) orientation x(3)
    % a_x, a_y => first host b_x,b_y => second host x,y number x(5) x(6)
    
    a_host = comb{i}(temp,1);
    b_host = comb{i}(temp,2);
    a_class = host_class(a_host);
    b_class = host_class(b_host);
   
    a_x = 3*a_host-2; a_y = 3*a_host-1;
    b_x = 3*b_host-2; b_y = 3*b_host-1;
    a_th = 3*a_host; b_th = 3*b_host;
    
    % Bore sight vectors of two hosts for a target
    bore1_ax = R*cos(x(a_th));
    bore1_ay = R*sin(x(a_th));    
    
    bore1_bx = R*cos(x(b_th));
    bore1_by = R*sin(x(b_th));       
    slope_bore_a = bore1_ay / bore1_ax;
    slope_bore_b = bore1_by / bore1_bx;
    
    %% Target vector
    % Target vector from two hosts
    atx = t(i,1) - x(a_x);
    aty = t(i,2) - x(a_y);
    btx = t(i,1) - x(b_x);
    bty = t(i,2) - x(b_y);
    
    % range to target
    a_range = norm([atx aty]);
    b_range = norm([btx bty]);  
    slope_ta = aty / atx;
    slope_tb = bty / btx;   
    
    if a_class == 1 % If the host is omnicam
        theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
        a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
        a_error = 1/a_scale;
    else % Or the host is ptz camera
        % target size compared to the coverage area
        a_scale = ptz_f * wt / a_range / ptz_ws;
        a_error = 1/a_scale;
    end
    

    if b_class == 1 % If the host is omnicam
        theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
        b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
        b_error = 1/b_scale;
    else % Or the host is ptz camera
        % target size compared to the coverage area
        b_scale = ptz_f * wt / b_range / ptz_ws;
        b_error = 1/b_scale;
    end
    
    error_size = norm([a_error b_error]);

    % Angle between two target vectors (should be close to 90 deg)
    psi = atan(abs((slope_ta - slope_tb)/(1 + slope_ta*slope_tb)));

    % Objective function (Maximize baseline of drones of each target)
    fun = fun + error_size*a_range*b_range/sin(psi);
    
    %% Future Target vector
    % Target vector from two hosts
    atx = t_future(i,1) - x(a_x);
    aty = t_future(i,2) - x(a_y);
    btx = t_future(i,1) - x(b_x);
    bty = t_future(i,2) - x(b_y);
    
    % range to target
    a_range = norm([atx aty]);
    b_range = norm([btx bty]);  
    slope_ta = aty / atx;
    slope_tb = bty / btx;
  
    if a_class == 1 % If the host is omnicam
        theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
        a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
        a_error = 1/a_scale;
    else % Or the host is ptz camera
        % target size compared to the coverage area
        a_scale = ptz_f * wt / a_range / ptz_ws;
        a_error = 1/a_scale;
    end
    

    if b_class == 1 % If the host is omnicam
        theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
        b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
        b_error = 1/b_scale;
    else % Or the host is ptz camera
        % target size compared to the coverage area
        b_scale = ptz_f * wt / b_range / ptz_ws;
        b_error = 1/b_scale;
    end
    
    error_size = norm([a_error b_error]);

    % Angle between two target vectors (should be close to 90 deg)
    psi = atan(abs((slope_ta - slope_tb)/(1 + slope_ta*slope_tb)));

    % Objective function (Maximize baseline of drones of each target)
    fun = fun + error_size*a_range*b_range/sin(psi);
    
%     %% Middle Target vector
%     % Target vector from two hosts
%     atx = t_middle(i,1) - x(a_x);
%     aty = t_middle(i,2) - x(a_y);
%     btx = t_middle(i,1) - x(b_x);
%     bty = t_middle(i,2) - x(b_y);
%     
%     % range to target
%     a_range = norm([atx aty]);
%     b_range = norm([btx bty]);  
%     
%     if a_class == 1 % If the host is omnicam
% 
%         slope_bore_a = bore1_ay / bore1_ax;
%         slope_ta = aty / atx;
%         theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
%         
%         a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
%         a_error = 1/a_scale;
%     else % Or the host is ptz camera
%         
%         % target size compared to the coverage area
%         a_scale = ptz_f * wt / a_range / ptz_ws;
%         a_error = 1/a_scale;
%         
%     end
%     
% 
%     if b_class == 1 % If the host is omnicam
%         
%         slope_bore_b = bore1_by / bore1_bx;
%         slope_tb = bty / btx;
%         theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
% 
%         b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
%         b_error = 1/b_scale;
%     else % Or the host is ptz camera
%         
%         % target size compared to the coverage area
%         b_scale = ptz_f * wt / b_range / ptz_ws;
%         b_error = 1/b_scale;
%     end
%     
%     error_size = norm([a_error b_error]);
% 
%     % Angle between two target vectors (should be close to 90 deg)
%     psi = atan(abs((slope_ta - slope_tb)/(1 + slope_ta*slope_tb)));
% 
%     % Objective function (Maximize baseline of drones of each target)
%     fun = fun + error_size*a_range*b_range/sin(psi);
end



figure;
plot(t(:,1),t(:,2),'ro');
hold on;     
xlim([-300,300]);
ylim([-300,300]);

temp_optimal_host2 = [];
for i=1:size(x,2)/3
    hx = 3*i - 2; hy = 3*i-1; hth = 3*i;
    temp_optimal_host2 = [temp_optimal_host2; x(hx), x(hy), x(hth)]; 
        
    if host_class(i) == 1 % if omnicam
        safety_limit = [temp_optimal_host2(i,1)+detectable_range_per_angle.*cos(safety_limit_angle+temp_optimal_host2(i,3)+pi/2); temp_optimal_host2(i,2)+detectable_range_per_angle.*sin(safety_limit_angle+temp_optimal_host2(i,3)+pi/2)];
        p_safe{i} = plot(safety_limit(1,:), safety_limit(2,:));
        plot([temp_optimal_host2(i,1)+R*cos(temp_optimal_host2(i,3)),temp_optimal_host2(i,1)+R*cos(temp_optimal_host2(i,3)+pi)],...
            [temp_optimal_host2(i,2)+R*sin(temp_optimal_host2(i,3)),temp_optimal_host2(i,2)+R*sin(temp_optimal_host2(i,3)+pi)]);
        hold on;
    else % if ptz
        plot([temp_optimal_host2(i,1)+ptz_R*cos(temp_optimal_host2(i,3)+ptz_fov),temp_optimal_host2(i,1)],...
            [temp_optimal_host2(i,2)+ptz_R*sin(temp_optimal_host2(i,3)+ptz_fov),temp_optimal_host2(i,2)]);
        hold on;
        plot([temp_optimal_host2(i,1)+ptz_R*cos(temp_optimal_host2(i,3)-ptz_fov),temp_optimal_host2(i,1)],...
        [temp_optimal_host2(i,2)+ptz_R*sin(temp_optimal_host2(i,3)-ptz_fov),temp_optimal_host2(i,2)]);
        hold on;
        plot([temp_optimal_host2(i,1)+ptz_R*cos(temp_optimal_host2(i,3)+ptz_fov),temp_optimal_host2(i,1)+ptz_R*cos(temp_optimal_host2(i,3)-ptz_fov)],...
        [temp_optimal_host2(i,2)+ptz_R*sin(temp_optimal_host2(i,3)+ptz_fov),temp_optimal_host2(i,2)+ptz_R*sin(temp_optimal_host2(i,3)-ptz_fov)]);
    end
    t1{i} = text(temp_optimal_host2(i,1),  temp_optimal_host2(i,2)+1,sprintf(': %d',i),'FontSize', 8);
    hold on;
end
p_host = plot(temp_optimal_host2(:,1), temp_optimal_host2(:,2), 'b*');