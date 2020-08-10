close all
clear all
global x x_star t R comb gamma target_num drone_num t_future t_middle x_initial ini_drone_num protection_R ini_host detectable_range_per_angle sim_time ptz_fov wt ws f D ptz_ws ptz_resol ptz_f host_class;
%objectve function: input coefficients
drone_num = 3;
ini_drone_num = drone_num;
max_target_per_drone =1;
R = 100;              % Range to threat to optimize
protection_R = 100;   % Host distances from the center
min_threat_R = 5;    % Threat min range from the host
max_threat_R = 100;  % Threat max range from the host

f = 0.001;
wt = 1;
ws = 0.003;
D = [-0.017496068223756347, -0.003243014339251435, 0.000839223410670477, -0.0004219234265229322];
ptz_ws = 0.0048;
ptz_resol = 1280;
ptz_f = 0.01;
ptz_R = ptz_f/20*wt/ptz_ws*ptz_resol;
ptz_fov = atan(ptz_ws/ptz_f);

    
base_station = [-300,-300];
drone_control_limit = 100;
detectable_range_per_angle = [8.62470502395184e-15,2.46449577912694,4.88628737344202,7.26572571782331,9.60319633362353,11.8991138568130,14.1539171083770,16.3680646608851,18.5420308589516,20.6763022547730,22.7713744230886,24.8277491228006,26.8459317751295,28.8264292305983,30.7697477993547,32.6763915213703,34.5468606549207,36.3816503634658,38.1812495826209,39.9461400503634,41.6767954849544,43.3736808962835,45.0372520174857,46.6679548447233,48.2662252739985,49.8324888247551,51.3671604408586,52.8706443603071,54.3433340457405,55.7856121684673,57.1978506393427,58.5804106803921,59.9336429316024,61.2578875877855,62.5534745608730,63.8207236634151,65.0599448094500,66.2714382292654,67.4554946949128,68.6123957536404,69.7424139667036,70.8458131512745,71.9228486234214,72.9737674403587,73.9988086403780,74.9982034790703,75.9721756606291,76.9209415631908,77.8447104573282,78.7436847169490,79.6180600219889,80.4680255524042,81.2937641730840,82.0954526093987,82.8732616131970,83.6273561191468,84.3578953913910,85.0650331605571,85.7489177512250,86.4096922000063,87.0474943644458,87.6624570229909,88.2547079663183,88.8243700803387,89.3715614212259,89.8963952828439,90.3989802569613,90.8794202866585,91.3378147133463,91.7742583178182,92.1888413557675,92.5816495881972,92.9527643071528,93.3022623571998,93.6302161530645,93.9366936938426,94.2217585741731,94.4854699927566,94.7278827585844,94.9490472952256,95.1490096435006,95.3278114628478,95.4854900316701,95.6220782469205,95.7376046231669,95.8320932913448,95.9055639973849,95.9580321008727,95.9895085738699,96,96,95.9895085738699,95.9580321008727,95.9055639973849,95.8320932913448,95.7376046231669,95.6220782469205,95.4854900316701,95.3278114628478,95.1490096435006,94.9490472952256,94.7278827585844,94.4854699927566,94.2217585741731,93.9366936938426,93.6302161530645,93.3022623571998,92.9527643071528,92.5816495881972,92.1888413557675,91.7742583178182,91.3378147133463,90.8794202866585,90.3989802569613,89.8963952828439,89.3715614212259,88.8243700803387,88.2547079663183,87.6624570229909,87.0474943644458,86.4096922000063,85.7489177512250,85.0650331605571,84.3578953913910,83.6273561191468,82.8732616131970,82.0954526093987,81.2937641730840,80.4680255524042,79.6180600219889,78.7436847169490,77.8447104573282,76.9209415631908,75.9721756606291,74.9982034790703,73.9988086403780,72.9737674403587,71.9228486234214,70.8458131512745,69.7424139667036,68.6123957536404,67.4554946949128,66.2714382292654,65.0599448094500,63.8207236634151,62.5534745608730,61.2578875877855,59.9336429316024,58.5804106803921,57.1978506393427,55.7856121684673,54.3433340457405,52.8706443603071,51.3671604408586,49.8324888247551,48.2662252739985,46.6679548447233,45.0372520174857,43.3736808962835,41.6767954849544,39.9461400503634,38.1812495826209,36.3816503634658,34.5468606549207,32.6763915213703,30.7697477993547,28.8264292305983,26.8459317751295,24.8277491228006,22.7713744230886,20.6763022547730,18.5420308589516,16.3680646608851,14.1539171083770,11.8991138568130,9.60319633362353,7.26572571782331,4.88628737344202,2.46449577912694,8.62470502395184e-15,8.62470502395184e-15,2.46449577912694,4.88628737344202,7.26572571782331,9.60319633362353,11.8991138568130,14.1539171083770,16.3680646608851,18.5420308589516,20.6763022547730,22.7713744230886,24.8277491228006,26.8459317751295,28.8264292305983,30.7697477993547,32.6763915213703,34.5468606549207,36.3816503634658,38.1812495826209,39.9461400503634,41.6767954849544,43.3736808962835,45.0372520174857,46.6679548447233,48.2662252739985,49.8324888247551,51.3671604408586,52.8706443603071,54.3433340457405,55.7856121684673,57.1978506393427,58.5804106803921,59.9336429316024,61.2578875877855,62.5534745608730,63.8207236634151,65.0599448094500,66.2714382292654,67.4554946949128,68.6123957536404,69.7424139667036,70.8458131512745,71.9228486234214,72.9737674403587,73.9988086403780,74.9982034790703,75.9721756606291,76.9209415631908,77.8447104573282,78.7436847169490,79.6180600219889,80.4680255524042,81.2937641730840,82.0954526093987,82.8732616131970,83.6273561191468,84.3578953913910,85.0650331605571,85.7489177512250,86.4096922000063,87.0474943644458,87.6624570229909,88.2547079663183,88.8243700803387,89.3715614212259,89.8963952828439,90.3989802569613,90.8794202866585,91.3378147133463,91.7742583178182,92.1888413557675,92.5816495881972,92.9527643071528,93.3022623571998,93.6302161530645,93.9366936938426,94.2217585741731,94.4854699927566,94.7278827585844,94.9490472952256,95.1490096435006,95.3278114628478,95.4854900316701,95.6220782469205,95.7376046231669,95.8320932913448,95.9055639973849,95.9580321008727,95.9895085738699,96,96,95.9895085738699,95.9580321008727,95.9055639973849,95.8320932913448,95.7376046231669,95.6220782469205,95.4854900316701,95.3278114628478,95.1490096435006,94.9490472952256,94.7278827585844,94.4854699927566,94.2217585741731,93.9366936938426,93.6302161530645,93.3022623571998,92.9527643071528,92.5816495881972,92.1888413557675,91.7742583178182,91.3378147133463,90.8794202866585,90.3989802569613,89.8963952828439,89.3715614212259,88.8243700803387,88.2547079663183,87.6624570229909,87.0474943644458,86.4096922000063,85.7489177512250,85.0650331605571,84.3578953913910,83.6273561191468,82.8732616131970,82.0954526093987,81.2937641730840,80.4680255524042,79.6180600219889,78.7436847169490,77.8447104573282,76.9209415631908,75.9721756606291,74.9982034790703,73.9988086403780,72.9737674403587,71.9228486234214,70.8458131512745,69.7424139667036,68.6123957536404,67.4554946949128,66.2714382292654,65.0599448094500,63.8207236634151,62.5534745608730,61.2578875877855,59.9336429316024,58.5804106803921,57.1978506393427,55.7856121684673,54.3433340457405,52.8706443603071,51.3671604408586,49.8324888247551,48.2662252739985,46.6679548447233,45.0372520174857,43.3736808962835,41.6767954849544,39.9461400503634,38.1812495826209,36.3816503634658,34.5468606549207,32.6763915213703,30.7697477993547,28.8264292305983,26.8459317751295,24.8277491228006,22.7713744230886,20.6763022547730,18.5420308589516,16.3680646608851,14.1539171083770,11.8991138568130,9.60319633362353,7.26572571782331,4.88628737344202,2.46449577912694,8.62470502395184e-15]; 


target_num = 0;
T = 1/10;
max_V = 5*T; % per second
sim_time = 10/T; % seconds

%% ----------- INITIALIZE HOST / THREAT ----------- %%

drone_position_angle = 2*pi/drone_num;
host_class = zeros(1,50);
for i = 1:drone_num
    ini_host(i,:) = [protection_R*cos(drone_position_angle*i), protection_R*sin(drone_position_angle*i)];
    host_class(i) = 1;
end
th = 90 * pi/180;
rot = [cos(th) -sin(th); sin(th) cos(th)];
% for i = 1:drone_num
%     ini_host(i,3:6) = [cos(th)*ini_host(i,1) - sin(th)*(ini_host(i,2) + R), sin(th)*ini_host(i,1) + cos(th)*(ini_host(i,2) + R), ...
%         cos(th)*ini_host(i,1) - sin(th)*(ini_host(i,2) - R), sin(th)*ini_host(i,1) + cos(th)*(ini_host(i,2) - R)];
% end

t =[];

% Create random threats/state
for i = 1:drone_num
    temp_target_num = randi([1, max_target_per_drone]);
    target_num = target_num + temp_target_num;
    for j = 1:temp_target_num
        range = rand()*(max_threat_R-min_threat_R)+min_threat_R;
        angle = rand()*2*pi;
        t = [t; ini_host(i,1) + range*cos(angle), ini_host(i,2) + range*sin(angle), i, rand()*max_V, rand()*2*pi];
    end
end
% target_num = 2;
% t= [  -89.8993  -54.4976    1.0000    0.4670    0.8162
%   149.4535    3.7052    2.0000    0.1686    1.0190];

% t = [  -23.8083  -46.1738    1.0000         0    0.6202
%    84.2493  -33.3241    2.0000         0    4.5316];

% target_num = 3;
% t = [ -139.7219  -52.9861    1.0000    0    3.9948
%    93.7118  -24.0400    2.0000    0    0.7502
%    40.3578   19.3219    2.0000    0    4.1591];

% target_num = 3;
% t = [ -129.2136   -1.0940    1.0000         0    5.5977
%  -191.9333  -28.1026    1.0000         0    0.9380
%    97.7168   84.8374    2.0000         0    1.5301];

target_num = 3;
t = [  -79.0582  151.3935    1.0000    0.4751    0.2164
  -45.9849 -127.6547    2.0000    0.3976    1.1742
   71.3002  -37.6366    3.0000    0.3547    4.7418];

% Compute future positions 
t_future = [];
t_middle = [];
for i = 1:target_num
    for j = 1:sim_time
        future_t(j,1,i) = t(i,1) + j*t(i,4)*cos(t(i,5));
        future_t(j,2,i) = t(i,2) + j*t(i,4)*sin(t(i,5));
    end
    t_future = [t_future; future_t(end,1,i) future_t(end,2,i), t(i,3), t(i,4), t(i,5)];
    t_middle = [t_middle; future_t(end/2,1,i) future_t(end/2,2,i), t(i,3), t(i,4), t(i,5)];
end



%% ---------- INITIAL PLOT ------------- %%

p_host = plot(ini_host(:,1), ini_host(:,2), 'b*');
hold on;
plot(t(:,1),t(:,2),'ro');
hold on;     
plot(t_future(:,1),t_future(:,2),'go');
hold on;
for i = 1:target_num
    p_threat_path{i} = plot([t(i,1),t_future(i,1)],[t(i,2),t_future(i,2)],'r');
end

safety_limit_angle = linspace(0,2*pi,360);
for i=1:drone_num
    t1{i} = text(ini_host(i,1),  ini_host(i,2)+1,sprintf(': %d',i),'FontSize', 8);
    hold on;
    safety_limit = [ini_host(i,1)+detectable_range_per_angle.*cos(safety_limit_angle); ini_host(i,2)+detectable_range_per_angle.*sin(safety_limit_angle)];
    p_safe{i} = plot(safety_limit(1,:), safety_limit(2,:),'b.');
    ori_host{i} = plot([ini_host(i,1)+R*cos(th), ini_host(i,1)+R*cos(th+pi)], [ini_host(i,2)+R*sin(th),ini_host(i,2)+R*sin(th+pi)]);
end
xlim([-300,300]);
ylim([-300,300]);



%% ------------ SOLVE FOR THE OPTIMAL POSITION -------------- %%
t
pause;
x_initial = [];
for i = 1:drone_num
    x_initial = [x_initial ini_host(i,1) ini_host(i,2), th];
end
optimal_pcv_placement_new_combi;

% plot(x_star(1),x_star(2),'b*');

% for i = 1:target_num
%     v1x = R*cos(x(3));
%     v1y = R*sin(x(3));    
%     v2x = R*cos(x(3)+pi);
%     v2y = R*sin(x(3)+pi);
%     vtx = t(i,1) - x(1);
%     vty = t(i,2) - x(2);
% 
%     theta = min(acos((v1x*vtx+v1y*vty)/norm([v1x v1y])/norm([vtx vty])),...
%         acos((v2x*vtx+v2y*vty)/norm([v2x v2y])/norm([vtx vty])));
% 
%     range = norm([t(i,1)-x(1) t(i,2)-x(2)]);
%     scale=(theta+D(1)*theta^3+D(2)*theta^5+D(3)*theta^7+D(4)*theta^9)./tan(theta);
%     
%     f/range*wt*1440/ws*scale
%     
% end

set(p_host,'Visible','off');

for i=1:ini_drone_num
    set(t1{i},'Visible','off');
    set(p_safe{i},'Visible','off');
    set(ori_host{i},'Visible','off');
end

temp_optimal_host = [];
for i=1:size(x_star,2)/3
    hx = 3*i - 2; hy = 3*i-1; hth = 3*i;
    temp_optimal_host = [temp_optimal_host; x_star(hx), x_star(hy), x_star(hth)]; 
        
    if host_class(i) == 1 % if omnicam
        safety_limit = [temp_optimal_host(i,1)+detectable_range_per_angle.*cos(safety_limit_angle+temp_optimal_host(i,3)+pi/2); temp_optimal_host(i,2)+detectable_range_per_angle.*sin(safety_limit_angle+temp_optimal_host(i,3)+pi/2)];
        p_safe{i} = plot(safety_limit(1,:), safety_limit(2,:));
        plot([temp_optimal_host(i,1)+R*cos(temp_optimal_host(i,3)),temp_optimal_host(i,1)+R*cos(temp_optimal_host(i,3)+pi)],...
            [temp_optimal_host(i,2)+R*sin(temp_optimal_host(i,3)),temp_optimal_host(i,2)+R*sin(temp_optimal_host(i,3)+pi)]);
        hold on;
    else % if ptz
        plot([temp_optimal_host(i,1)+ptz_R*cos(temp_optimal_host(i,3)+ptz_fov),temp_optimal_host(i,1)],...
            [temp_optimal_host(i,2)+ptz_R*sin(temp_optimal_host(i,3)+ptz_fov),temp_optimal_host(i,2)],'b');
        hold on;
        plot([temp_optimal_host(i,1)+ptz_R*cos(temp_optimal_host(i,3)-ptz_fov),temp_optimal_host(i,1)],...
        [temp_optimal_host(i,2)+ptz_R*sin(temp_optimal_host(i,3)-ptz_fov),temp_optimal_host(i,2)],'b');
        hold on;
        plot([temp_optimal_host(i,1)+ptz_R*cos(temp_optimal_host(i,3)+ptz_fov),temp_optimal_host(i,1)+ptz_R*cos(temp_optimal_host(i,3)-ptz_fov)],...
        [temp_optimal_host(i,2)+ptz_R*sin(temp_optimal_host(i,3)+ptz_fov),temp_optimal_host(i,2)+ptz_R*sin(temp_optimal_host(i,3)-ptz_fov)],'r');
    end
    t1{i} = text(temp_optimal_host(i,1),  temp_optimal_host(i,2)+1,sprintf(': %d',i),'FontSize', 8);
    hold on;
end
p_host = plot(temp_optimal_host(:,1), temp_optimal_host(:,2), 'b*');




%% ------------ SOLVE FOR THE OPTIMAL POSITION -------------- %%

% This just solves problem based on only threat position
% The initial host position is not even included in the computation.
% I just connected the initial drone position to computed drone positions.
% pause;
% optimal_pcv_placement_v2;
% 
% 
% 
% optimal_host = [];
% for i=1:size(x,2)/2
%     hx = 2*i - 1; hy = 2*i;
%     optimal_host = [optimal_host; x(hx), x(hy)]; 
% end
% 
% %% ----------- PLOT THREAT STATE ----------- %%
% 
% plot(t(:,1),t(:,2),'ro');
% hold on;
% for i=1:target_num
%     t2{i}=text(t(i,1), t(i,2)+1,sprintf('%d',i),'FontSize', 8);
%     hold on;
% end
% 
%     for i=1:ini_drone_num
%         set(t1{i},'Visible','off');
%         set(p_safe{i},'Visible','off');
%     end
%     set(p_host,'Visible','off');
% 
% %% ------------------- PD controller ------------------- %%
% p_error = 10000000;
% host_state = [ini_host; zeros(size(optimal_host,1) - size(ini_host,1),2) + base_station];
% safety_limit_angle = linspace(0,360,360);
% 
% 
% while max(max(abs(p_error))) > 0.1
% 
%     p_host = plot(host_state(:,1), host_state(:,2), 'b*');
%     hold on;
%     
%     for i=1:drone_num
%         t1{i}=text(host_state(i,1),  host_state(i,2)+1,sprintf(': %d',i),'FontSize', 8);
%         hold on;
%         safety_limit = [host_state(i,1)+R.*cos(safety_limit_angle); host_state(i,2)+R.*sin(safety_limit_angle)];
%         p_safe{i} = plot(safety_limit(1,:), safety_limit(2,:),'b.');
%     end
%   
%     drawnow;
% 
%     
%     drone_control_limit = 100;
%     Kp = 0.1; Kd = 0.1; % PID Controller parametrs
%     state = host_state;
%     goal = optimal_host;
%     
%     p_error_prev = 0;
%     p_error = goal - state;
%     d_error = p_error - p_error_prev;
%     control = Kp*p_error + Kd*d_error;
%     control = min(drone_control_limit, control);
%     host_state = host_state + control;
% 
%     for i=1:drone_num
%         set(t1{i},'Visible','off');
%         set(p_safe{i},'Visible','off');
%     end
%     set(p_host,'Visible','off');
% 
% end
% 
% 
% p_host = plot(hostd_state(:,1), host_state(:,2), 'b*');
% hold on;
% 
% for i=1:drone_num
%     t1{i}=text(host_state(i,1),  host_state(i,2)+1,sprintf(': %d',i),'FontSize', 8);
%     hold on;
%     safety_limit = [host_state(i,1)+R.*cos(safety_limit_angle); host_state(i,2)+R.*sin(safety_limit_angle)];
%     p_safe{i} = plot(safety_limit(1,:), safety_limit(2,:),'b.');
% end


