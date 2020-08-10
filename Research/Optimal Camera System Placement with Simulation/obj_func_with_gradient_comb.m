function fun = obj_func_with_gradient_comb(x)

global gamma target_num comb drone_num R t t_future t_middle sim_time host_class wt ws f D ptz_ws ptz_resol ptz_f


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
