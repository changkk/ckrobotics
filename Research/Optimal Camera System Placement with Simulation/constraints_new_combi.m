function [c,ceq] = constraints_new_combi(x)
global t t_future t_middle R comb gamma target_num ini_drone_num protection_R ptz_fov host_class wt ws f D ptz_ws ptz_resol ptz_f;

c = []; ceq = [];


% For covering theats
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

    
    %% Current target vector
    
    % Target vector from two hosts
    atx = t(i,1) - x(a_x);
    aty = t(i,2) - x(a_y);      
    btx = t(i,1) - x(b_x);
    bty = t(i,2) - x(b_y); 
    slope_ta = aty / atx;
    slope_tb = bty / btx;

    % range to target
    a_range = norm([atx aty]);   
    b_range = norm([btx bty]);  

    if a_class == 1 % If the host is omnicam
        % The smallest angle between the target vector and the boresight.
        theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
        a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
        target_px = f/a_range*wt*1440/ws*a_scale;
        
        c = [c; -target_px + 5];
        c = [c; -a_range + 40];
    else  % Or PTZ camera
        theta = acos((bore1_ax*atx+bore1_ay*aty)/norm([atx aty])/norm([bore1_ax bore1_ay]));
        target_px = ptz_f/a_range*wt/ptz_ws*ptz_resol;
        
        c = [c; theta - ptz_fov];
        c = [c; -target_px + 20];
        c = [c; -a_range + 40];
    end
   
    if b_class == 1
        theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
        b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
        target_px = f/b_range*wt*1440/ws*b_scale;
        
        c = [c; -target_px + 5];
        c = [c; -b_range + 40];
    else
        theta = acos((bore1_bx*btx+bore1_by*bty)/norm([btx bty])/norm([bore1_bx bore1_by]));
        target_px = ptz_f/b_range*wt/ptz_ws*ptz_resol;
        
        c = [c; theta - ptz_fov];
        c = [c; -target_px + 20]; 
        c = [c; -b_range + 40];
    end
    

    
    %% Future target vector
    
    
    % Target vector from two hosts
    atx = t_future(i,1) - x(a_x);
    aty = t_future(i,2) - x(a_y);      
    btx = t_future(i,1) - x(b_x);
    bty = t_future(i,2) - x(b_y); 
    slope_ta = aty / atx;
    slope_tb = bty / btx;
    
    % range to target
    a_range = norm([atx aty]);   
    b_range = norm([btx bty]);  

    if a_class == 1 % If the host is omnicam
        % The smallest angle between the target vector and the boresight.
        theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
        a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
        target_px = f/a_range*wt*1440/ws*a_scale;
        
        c = [c; -target_px + 5];
        c = [c; -a_range + 40];
    else  % Or PTZ camera
        theta = acos((bore1_ax*atx+bore1_ay*aty)/norm([atx aty])/norm([bore1_ax bore1_ay]));
        target_px = ptz_f/a_range*wt/ptz_ws*ptz_resol;
        
        c = [c; theta - ptz_fov];
        c = [c; -target_px + 20];
        c = [c; -a_range + 40];
    end
   
    if b_class == 1
        theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
        b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
        target_px = f/b_range*wt*1440/ws*b_scale;
        
        c = [c; -target_px + 5];
        c = [c; -b_range + 40];
    else
        theta = acos((bore1_bx*btx+bore1_by*bty)/norm([btx bty])/norm([bore1_bx bore1_by]));
        target_px = ptz_f/b_range*wt/ptz_ws*ptz_resol;
        
        c = [c; theta - ptz_fov];
        c = [c; -target_px + 20]; 
        c = [c; -b_range + 40];
    end
   
%     %% Middle target vector
%     
%     
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
%         % The smallest angle between the target vector and the boresight.
%         slope_bore_a = bore1_ay / bore1_ax;
%         slope_ta = aty / atx;
%         theta1 = atan(abs((slope_ta - slope_bore_a)/(1 + slope_ta*slope_bore_a)));
%         a_scale = (theta1+D(1)*theta1^3+D(2)*theta1^5+D(3)*theta1^7+D(4)*theta1^9)./tan(theta1);
%         target_px = f/a_range*wt*1440/ws*a_scale;
%         
%         c = [c; -target_px + 5];
%         c = [c; -a_range + 40];
%     else  % Or PTZ camera
%         theta = abs(acos((bore1_ax*atx+bore1_ay*aty)/norm([atx aty])/norm([bore1_ax bore1_ay])));
%         target_px = ptz_f/a_range*wt/ptz_ws*ptz_resol;
%         
%         c = [c; theta - ptz_fov];
%         c = [c; -target_px + 20];
%         c = [c; -a_range + 40];
%     end
%    
%     if b_class == 1
%         slope_bore_b = bore1_by / bore1_bx;
%         slope_tb = bty / btx;
%         theta2 = atan(abs((slope_tb - slope_bore_b)/(1 + slope_tb*slope_bore_b)));
%         b_scale = (theta2+D(1)*theta2^3+D(2)*theta2^5+D(3)*theta2^7+D(4)*theta2^9)./tan(theta2);
%         target_px = f/b_range*wt*1440/ws*b_scale;
%         
%         c = [c; -target_px + 5];
%         c = [c; -b_range + 40];
%     else
%         theta = abs(acos((bore1_bx*btx+bore1_by*bty)/norm([btx bty])/norm([bore1_bx bore1_by])));
%         target_px = ptz_f/b_range*wt/ptz_ws*ptz_resol;
%         
%         c = [c; theta - ptz_fov];
%         c = [c; -target_px + 20]; 
%         c = [c; -b_range + 40];
%     end
    
    %%
    % Distance between hosts
    distance = norm([x(a_x) - x(b_x), x(a_y) - x(b_y)]);
    
    % two points (current, future) should be on the same side.
    c = [c; 
        -distance + 40];
%         -(t_future(i,2) - x(a_y))*(t(i,2) - x(a_y));
%         -(t_future(i,2) - x(b_y))*(t(i,2) - x(b_y))];
    
%     c = [c; -psi(x) + 0];
%     c = [c; psi(x) - pi];
end


    
