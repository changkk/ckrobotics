for y=1:2
    clearvars -except p y output_costs output_numiters 
    a_star = [0 , 1];
    a_star_flag = a_star(y);
    for p=1:3
        clearvars -except p y output_costs output_numiters a_star_flag a_star
            
        filename_input = ["input_1.txt", "input_2.txt", "input_3.txt"];
        filename_coords = ["coords_1.txt", "coords_2.txt", "coords_3.txt"];

        delimiterIn = ' ';
        headerlinesIn = 3;
        A = importdata(filename_input(p),delimiterIn,headerlinesIn);

        data=A.data;
        b=regexp(A.textdata,'\d+(\.)?(\d+)?','match');
        goal=str2double([b{:}]);

        delimiterIn = ' ';
        headerlinesIn = 0;
        coor = importdata(filename_coords(p),delimiterIn,headerlinesIn);

        finding=ismember(data(:,1),goal(2));
        vertex_index=find(finding);
        size_vertex_index = size(vertex_index);

        V(1:goal(1),1) = linspace(1,goal(1),goal(1));
        V(1:goal(1),2) = ones(goal(1),1)*10000000;
        O=[];

        goal_position = [coor(goal(3),1), coor(goal(3),2) ];

        C = [6, 6, 0, 0];

        for i=1:size_vertex_index(1)
            vertex(i,1) = data(vertex_index(i),1);
            vertex(i,2) = data(vertex_index(i),2);
            vertex_position = [coor(vertex(i,2),1), coor(vertex(i,2),2)];
            if(a_star_flag == 0)
                distance_to_goal = 0;
            else
                distance_to_goal = norm(vertex_position - goal_position);                
            end
            vertex(i,3) = data(vertex_index(i),3);
            vertex(i,4) = vertex(i,3) + distance_to_goal;
        end
            size_O = size(O);
            vertex_size = size(vertex);
            % save optimal values
            for j=1:vertex_size(1)
                if V(vertex(j,2),2) > vertex(j,3)
                    V(vertex(j,2),2) = vertex(j,3); % save optimal value
                    V(vertex(j,2),3) = vertex(j,1); % save where this value came
                    O(size_O(1)+j,:) = vertex(j,:);
                end
            end

        O = sortrows(vertex,4);
        size_O = size(O);
        vertex_index = [];
        vertex = [];
        flag = 0;
        t=0;

        while flag == 0
        t=t+1;
            % expansion
            finding=ismember(data(:,1),O(1,2));
            vertex_index=find(finding);
            size_vertex_index = size(vertex_index);
            size_C = size(C);

            C(size_C(1)+1,:) = O(1,:); % add to closed list
            O(1,:) = []; % delete from open list

            for i=1:size_vertex_index(1)
                vertex(i,1) = data(vertex_index(i),1);
                vertex(i,2) = data(vertex_index(i),2);
                vertex_position = [coor(vertex(i,2),1), coor(vertex(i,2),2)];
                if(a_star_flag == 0)
                    distance_to_goal = 0;
                else
                    distance_to_goal = norm(vertex_position - goal_position);                
                end
                vertex(i,3) = data(vertex_index(i),3) + C(end,3);
                vertex(i,4) = vertex(i,3) + distance_to_goal;
            end

            size_O = size(O);
            vertex_size = size(vertex);

            for i=1:vertex_size(1)
                    O(size_O(1)+i,:) = vertex(i,:);
            end 

            size_C = size(C);
            for i=1:size_C(1)
                finding_member_of_C=ismember(O(:,2),C(i,2));
                finding_member_of_C_index=find(finding_member_of_C);
                O(finding_member_of_C_index,:)=[];
            end

            O = sortrows(O,4);
            vertex_index = [];
            vertex = [];


            % terminate condition
            finding_goal_in_C=ismember(C(:,2),goal(3));
            finding_goal_in_C_index=find(finding_goal_in_C);
            if isempty(finding_goal_in_C_index) == 0 % Wait until Xg be in C
                flag = 1;
            end

        end

        output_costs(p,y) = C(end,3);
        output_numiters(p,y) = t+1;

    end
end

T = table(output_costs(:,1),output_costs(:,2));
writetable(T,'output_costs.txt');
type output_costs.txt;

T2 = table(output_numiters(:,1),output_numiters(:,2));
writetable(T,'output_numiters.txt');
type output_numiters.txt;