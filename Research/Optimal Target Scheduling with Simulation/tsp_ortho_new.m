function target_order = tsp_ortho_new(number, gimbal_position, X, k, first_number, risk, w_rv, df)

% data = [];
% % Compute each angle between current camera orientaton and targets
% for j = 1:number
%     data(j,1)=0;
%     data(j,2)=j;
%     oa = sqrt(gimbal_position(1)^2 + gimbal_position(2)^2);
%     ob = sqrt(X(j,1)^2 + X(j,2)^2);
%     data(j,3)= acos(dot([gimbal_position(1) gimbal_position(2)],[X(j,1) X(j,2)])/(oa*ob))*180/pi;
% end
% 
% % Compute each angle between all the targets
% for i = 1:number
%     for j = 1:number
%         data(number*i+j,1)=i;
%         data(number*i+j,2)=j;
%         oa = sqrt(X(i,1)^2 + X(i,2)^2);
%         ob = sqrt(X(j,1)^2 + X(j,2)^2);
%         data(number*i+j,3) = acos(dot([X(i,1) X(i,2)],[X(j,1) X(j,2)])/(oa*ob))*180/pi;
%         if i==j % self cost = 0
%             data(number*i+j,3) = 0;
%         end
%     end
% end
% 
% % Delete distance 0 (self)
% self=ismember(data(:,3),0);
% self_index = find(self);
% data(self_index,:) = [];
% data(:,3) = data(:,3)/180;
    
    
if first_number == 0    
    for i=1:number
        for j=i+1:number+1
            if i-1==0
                oa = sqrt(gimbal_position(1)^2 + gimbal_position(2)^2);
                ob = sqrt(X(j-1,1)^2 + X(j-1,2)^2);
                distance(i,j) = acos(dot([gimbal_position(1) gimbal_position(2)],[X(j-1,1) X(j-1,2)])/(oa*ob))*180/pi;
                distance(j,i) = distance(i,j);
            else
                oa = sqrt(X(i-1,1)^2 + X(i-1,2)^2);
                ob = sqrt(X(j-1,1)^2 + X(j-1,2)^2);
                distance(i,j) = acos(dot([X(i-1,1) X(i-1,2)],[X(j-1,1) X(j-1,2)])/(oa*ob))*180/pi;
                distance(j,i) = distance(i,j);
            end

        end
    end
else
    for i=1:number-1
        for j=i+1:number
            oa = sqrt(X(i,1)^2 + X(i,2)^2);
            ob = sqrt(X(j,1)^2 + X(j,2)^2);
            distance(i,j) = acos(dot([X(i,1) X(i,2)],[X(j,1) X(j,2)])/(oa*ob))*180/pi;
            distance(j,i) = distance(i,j);
        end
    end
end

distance = distance/180;
    
for i=1:number
    target(i) = i;
end

% if(first_number ~= 0)
%     target(target==first_number) = [];
% end

P = perms(target);
%P = [first_number*ones(size(P,1),1) P];

min_cost = 1000000;
for i=1:size(P,1)
    cost = 0;
    
    if first_number == 0
        
        for j=2:size(P,2)
            cost = cost + distance(P(i,j-1)+1,P(i,j)+1)*(1-w_rv) + risk(P(i,j)) * (j+200) * w_rv;
            if(j==size(P,2))
                cost = cost + distance(P(i,j-1)+1,P(i,j)+1)*(1-w_rv);
            end
        end
        cost_list_main(i) = cost;
        min_cost = min(min_cost,cost);
        if min_cost == cost
            min_inx = i;
        end    
        
    else
    
        for j=2:size(P,2)
            cost = cost + distance(P(i,j-1),P(i,j))*(1-w_rv) + risk(P(i,j)) * (j+200) * w_rv ;
            if(j==size(P,2))
                cost = cost + distance(P(i,j-1),P(i,j))*(1-w_rv);
            end
        end
        min_cost = min(min_cost,cost);
        if min_cost == cost
            min_inx = i;
        end
    
    end
end

target_order=P(min_inx,:);

        