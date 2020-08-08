clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DRAW OBSTACLES %%%%%%%%%%%%%%%%%%%%%%%%

tmp_data = importdata('input_rrt.txt',',');
size_tmp_data=size(tmp_data);
tmp_data2=cell2mat(tmp_data(1));
start=str2num(tmp_data2);
tmp_data2=cell2mat(tmp_data(2));
goal=str2num(tmp_data2);

tmp_data2=cell2mat(tmp_data(3));
obstacle1=[str2num(tmp_data2)];
obstacle1 = [obstacle1 obstacle1(1) obstacle1(2)];

tmp_data2=cell2mat(tmp_data(4));
obstacle2=[str2num(tmp_data2)];
obstacle2 = [obstacle2 obstacle2(1) obstacle2(2)];

tmp_data2=cell2mat(tmp_data(5));
obstacle3=[str2num(tmp_data2)];
obstacle3= [obstacle3 obstacle3(1) obstacle3(2)];

tmp_data2=cell2mat(tmp_data(6));
obstacle4=[str2num(tmp_data2)];
obstacle4 = [obstacle4 obstacle4(1) obstacle4(2)];

tmp_data2=cell2mat(tmp_data(7));
obstacle5=[str2num(tmp_data2)];
obstacle5 = [obstacle5 obstacle5(1) obstacle5(2)];


size_obstacle1 = size(obstacle1);
size_obstacle2 = size(obstacle2);
size_obstacle3 = size(obstacle3);
size_obstacle4 = size(obstacle4);
size_obstacle5 = size(obstacle5);


for i=1:2:size_obstacle1(2)-3
    plot(obstacle1(i),obstacle1(i+1),'k.');
    hold on;
    plot([obstacle1(i),obstacle1(i+2)],[obstacle1(i+1),obstacle1(i+3)],'r');
    hold on;
end
for i=1:2:size_obstacle2(2)-3
    plot(obstacle2(i),obstacle2(i+1),'k.');
    hold on;
    plot([obstacle2(i),obstacle2(i+2)],[obstacle2(i+1),obstacle2(i+3)],'r');
    hold on;
end
for i=1:2:size_obstacle3(2)-3
    plot(obstacle3(i),obstacle3(i+1),'k.');
    hold on;
    plot([obstacle3(i),obstacle3(i+2)],[obstacle3(i+1),obstacle3(i+3)],'r');
    hold on;
end
for i=1:2:size_obstacle4(2)-3
    plot(obstacle4(i),obstacle4(i+1),'k.');
    hold on;
    plot([obstacle4(i),obstacle4(i+2)],[obstacle4(i+1),obstacle4(i+3)],'r');
    hold on;
end
for i=1:2:size_obstacle5(2)-3
    plot(obstacle5(i),obstacle5(i+1),'k.');
    hold on;
    plot([obstacle5(i),obstacle5(i+2)],[obstacle5(i+1),obstacle5(i+3)],'r');
    hold on;
end
xlim([-12 12]);
ylim([-12 12]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DRAW OBSTACLES %%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%% PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
epsilon = 0.9;
destination_threshold = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Q = [start];
q_n = start;
arrived = 0;

plot(start(1),start(2),'r*');
plot(goal(1),goal(2),'b*');

while arrived == 0

    %%%%%%% Random alpha  
    alpha = [rand(1)*24-12 rand(1)*24-12];
    
    %%%%%%% find q_near
    
    size_Q = size(Q);
    for j = 1:size_Q(1)
        Q_distance(j) = sqrt((alpha(1)-Q(j,1))^2 + (alpha(2)-Q(j,2))^2);
    end
    [Q_distance_min, Q_distance_index] = min(Q_distance);
    q_n = Q(Q_distance_index,:);
    Q_distance = [];

    
    %%%%%%% if alpha is too far, normalize by epsilon
        
    distance_alpha = sqrt((q_n(1)-alpha(1))^2+(q_n(2)-alpha(2))^2);

    if distance_alpha > epsilon
        alpha(1) = q_n(1) + (alpha(1)-q_n(1))*epsilon / distance_alpha;
        alpha(2) = q_n(2) + (alpha(2)-q_n(2))*epsilon / distance_alpha;
    end
    
    %%%%% if alpha is inside obstacle, ignore it
    alpha_inobstacle1=maze_inobstacle(alpha,obstacle1);
    alpha_inobstacle2=maze_inobstacle(alpha,obstacle2);
    alpha_inobstacle3=maze_inobstacle(alpha,obstacle3);
    alpha_inobstacle4=maze_inobstacle(alpha,obstacle4);
    alpha_inobstacle5=maze_inobstacle(alpha,obstacle5);
    
    if alpha_inobstacle1==true || alpha_inobstacle2==true || alpha_inobstacle3==true || alpha_inobstacle4==true || alpha_inobstacle5==true 
        alpha_inobstacle = true;
    else
        alpha_inobstacle = false;
    end
    
    
    % if alpha is not inside obstacles, then start
    if alpha_inobstacle == false

%         plot(alpha(1),alpha(2),'+')
        hold on;

        % Check the collision
        intersec = maze_Collision_obstacle(q_n,alpha,obstacle1,obstacle2,obstacle3,obstacle4,obstacle5);
%         intersec = round(intersec,1);
        intersec = unique(intersec, 'rows');
        size_intersec = size(intersec);

        % if there is no collision, just connect.
        if isempty(intersec)==true
            q_s = alpha;
        else % if there is collision, pick the closest point to q_n and connect it.
            for j=1:size_intersec(1)
                distance(j) = sqrt((q_n(1)-intersec(j,1))^2 + (q_n(2)-intersec(j,2))^2);
            end
            [val,min_index]=min(distance);
                if size_intersec(1) == 1 && val < eps(10)
                            q_s = alpha;
                else
                    q_s=[intersec(min_index,1),intersec(min_index,2)];
%                     plot(intersec(:,1),intersec(:,2),'r*')
                    hold on;
                end
        end
        distance = [];
        q_s = round(q_s,1);
%         plot(q_s(1),q_s(2),'ro')
        plot([q_s(1), q_n(1)],[q_s(2),q_n(2)],'k');
        
        if abs(q_s - goal) < destination_threshold
            arrived = 1;
        end
        
        i = size_Q(1) + 1;
        Q(i,:) = q_s;
        Q_path(i,:) = [q_n q_s];
        Q = unique(Q,'rows');
        intersec=[];
        drawnow;
    end
end


size_q_path = size(Q_path);

next = Q_path(size_q_path(1), 1:2);
flag = 0;
k = 1;
path_save(1,:) = q_s;

while flag == 0
    
    k = k+1;
    path_save(k,:) = next;
    
    for i = 1:size_q_path(1)
        if abs(Q_path(i,3:4) - next) < eps(10000) 
            next = Q_path(i, 1:2);
            break;
        end
    end
    
    
    if abs(next - start) < eps(10000)
        flag = 1;
        k = k+1;
        path_save(k,:) = next;
    end
    
end

size_path_save = size(path_save);

for i=1:size_path_save(1)-1
    plot([path_save(i,1), path_save(i+1,1)],[path_save(i,2),path_save(i+1,2)],'r','LineWidth',3);
end