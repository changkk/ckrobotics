function target_order = tsp(number, gimbal_position, X, k, first_number)

    data = [];
    % Compute each angle between current camera orientaton and targets
    for j = 1:number
        data(j,1)=0;
        data(j,2)=j;
        oa = sqrt(gimbal_position(1)^2 + gimbal_position(2)^2);
        ob = sqrt(X(j,1)^2 + X(j,2)^2);
        data(j,3)= acos(dot([gimbal_position(1) gimbal_position(2)],[X(j,1) X(j,2)])/(oa*ob))*180/pi;
    end

    % Compute each angle between all the targets
    for i = 1:number
        for j = 1:number
            data(number*i+j,1)=i;
            data(number*i+j,2)=j;
            oa = sqrt(X(i,1)^2 + X(i,2)^2);
            ob = sqrt(X(j,1)^2 + X(j,2)^2);
            data(number*i+j,3) = acos(dot([X(i,1) X(i,2)],[X(j,1) X(j,2)])/(oa*ob))*180/pi;
            if i==j % self cost = 0
                data(number*i+j,3) = 0;
            end
        end
    end

    % Delete distance 0 (self)
    self=ismember(data(:,3),0);
    self_index = find(self);
    data(self_index,:) = [];

    % random target / cost array
    if k==1
        while_num = number+1;
    else
        while_num = number;
    end

    target_order = [first_number];
    
    while max(size(target_order)) < while_num

        % add next target to target order 
        % and check if the next target is inside the target order array 
        next_target_num = randi(number);
        next_target_check=ismember(target_order,next_target_num);
        next_target_check2 = find(next_target_check);

        % Unless next target is inside the target array,
        if isempty(next_target_check2)==true

            path_array_num = 1;
            path = [];
            path_save = [];
            path_cost = [];

            for p=1:max(size(target_order))
                path=[target_order(1:p) next_target_num target_order(p+1:end)];

                path_cost2 = 0;
                % find cost
                for i=1:max(size(path))-1
                    for j=1:max(size(data))
                        if abs(data(j,1)-path(i)) < eps(10000) && abs(data(j,2)-path(i+1)) < eps(10000)
                            path_cost2 = path_cost2 + data(j,3);
                            break;
                        end
                    end
                end

                path_save(path_array_num,:) = path;
                path_cost(path_array_num) = path_cost2;
                path_array_num = path_array_num + 1;

            end

            [cost_min_value, cost_min_array] = min(path_cost);
            target_order = path_save(cost_min_array,:);
        end
    end
        