global x x_star t R comb gamma target_num drone_num t_future t_middle x_initial host_class;

%% ----------- COMPUTATION START ----------- %%

fmin = 10000000000000;

host = ini_host;
drone_num = drone_num - 1;
x_star = [];
while isempty(x_star) == 1
    
    drone_num = drone_num + 1
    if host_class(drone_num) == 0
        host_class(drone_num) = round(rand())+1;
    end
    
%% ----------- comb EXAMPLE / EXPLANATION ---------- %%
%     comb{1} - for threat 1
%     1 2    -  host 1 and 2 observes threat 1
%     1 3    -  host 1 and 3 observes threat 1
%     The first col should be the host number that currently observing
%     threat 1
%     comb_size : the number of combinations of hosts for each threat
    

%% ----------- Target - Host assignment (For first host-target pair) ---------- %%

% If a host cannot cover initlally assigned targets, new host is added and assigned.

    for i = 1:drone_num
        if(size(find(t(:,3) == i),1) > 1) % If multiple targets are detected by a host
            
            t_idx = find(t(:,3) == i); % e.g.) t_idx = [2; 3] -> threat 1 is covered by host 2,3
            % If the host cannot cover assigned targets,
            if(check_covering(t_idx) == 0)
               % Assign new drone to the targets
               drone_num = drone_num + 1;
               if host_class(drone_num) == 0
                   host_class(drone_num) = round(rand+1);
               end
               for j = 2:size(t_idx,1)
                   t(t_idx(j),3) = drone_num;
                   t_middle(t_idx(j),3) = drone_num;
                   t_future(t_idx(j),3) = drone_num;
               end
            end
            
        end
    end
    
%% Can decrease drone number here (optimize the new drone number)

%% ----------- Possible combination generation (for second host - target pair) ---------- %%

    for i = 1:target_num
        idx = 1;
        for j = 1:drone_num
            if t(i,3)~=j % to avoid duplicate drone number
                comb{i}(idx,:) = [t(i,3) j]; % [initial_detecting_drone, another_drone]
                idx = idx + 1;
            end
        end
    end
    comb_size = size(comb{1},1);
    
%% ----------- Branch and Bound ---------- %%

    
    % Initialize queue [ host1  host2  idx ]
    % Here only parent node (1 and 2) is considered and later subnodes
    % (> host3) will be added if parent node is feasible.
    
    % bnb_bfs looks like
    % [drone1 drone2 comb_idx       for threat 1
    %  drone3 drone4 comb_idx       for target 2
    %  drone5 drone6 comb_idx]      for target 3
    
    bnb_bfs = CQueue;
    for i = 1:comb_size
        for j = 1:comb_size
            bnb_bfs.push([comb{1}(i,:), i; 
                          comb{2}(j,:), j]); % 
        end
    end
    
    % Start from parent, add parent + child
    while size(bnb_bfs) > 0
        gamma = zeros(comb_size,target_num);
%         count = count + 1

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % gamma     (v: bnb)
        % [ h1h2(t1)v h2h1(t2)v h3h1(t3)v ]
        % [ h1h3(t1) h2h3(t2) h3h2(t3) ]
        % [ h1h4(t1) h2h4(t2) h3h4(t3) ]
        % [ h1h5(t1) h2h5(t2) h3h5(t3) ]
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        cur = bnb_bfs.front();
        bnb_bfs.pop();
        % fesibility check : h1 -> t1t2 and h2 -> t1t2 possible? YES.
        bool_first_feas = check_first_feasible_new(cur);

        if bool_first_feas == 1
            if size(cur,1) < target_num
                % If the parent node is feasible, add all subnodes
                for i=1:comb_size
                    bnb_bfs.push([cur; comb{size(cur,1)+1}(i,:), i]);
                end
            else
                % If this is the last subnode and feasible, assign gamma
                for i = 1:target_num
                    gamma(cur(i,3), i) = 1;
                end
                % Find the optimal value using gamma
                [x,fval] = solve_fmincon_new_combi(x_initial);
                x
                gamma
%                 x_initial = x;
%                 save_vals(:,:,count) = [cur; fval 0 0];
                % If the solution is feasible, update
                if check_feasible_new_combi(x) == 1 && fval < fmin % non-feasible solution should be filtered
                    fmin = fval
                    gamma_star = gamma;
                    x_star = x;
                end
            end
        end

    end

end
% 
% figure;
% plot(t(:,1),t(:,2),'ro');
% hold on;
% 
% for i=1:target_num
%     t2{i}=text(t(i,1), t(i,2)+1,sprintf(': %d',i),'FontSize', 8);
%     hold on;
% end
% 
% safety_limit_angle = linspace(0,360,360);
% for i=1:drone_num
%     p_x = (2*i)-1; p_y = 2*i;
%     plot(x_star(p_x),x_star(p_y), 'b*');
%     t1{i}=text(x_star(p_x), x_star(p_y)+1,sprintf(': %d',i),'FontSize', 8);
%     hold on;
%     safety_limit = [x_star(p_x)+R.*cos(safety_limit_angle); x_star(p_y)+R.*sin(safety_limit_angle)];
%     plot(safety_limit(1,:), safety_limit(2,:),'.');
%     hold on;
% end
% 
% 
% xlim([-70,70]);
% ylim([-70,70]);
