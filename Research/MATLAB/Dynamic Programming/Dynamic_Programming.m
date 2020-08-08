for p=1:2
    clearvars -except p
    
        filename_input = ["input1.txt", "input2.txt"];

        delimiterIn = ' ';
        headerlinesIn = 3;
        A = importdata(filename_input(p),delimiterIn,headerlinesIn);

        data=A.data;
        b=regexp(A.textdata,'\d+(\.)?(\d+)?','match');
        goal=str2double([b{:}]);

    finding=ismember(data(:,2),goal(3));
    vertex_index=find(finding);
    size_vertex_index = size(vertex_index);

    V(1:goal(1),1) = linspace(1,goal(1),goal(1));
    V(1:goal(1),2) = ones(goal(1),1)*10000000;

    for i=1:size_vertex_index(1)
        vertex(i,1) = data(vertex_index(i),1);
        vertex(i,2) = data(vertex_index(i),3);
        V(vertex(i,1),2) = data(vertex_index(i),3); % Save Optimal values ('V') (initial point 1)
        V(vertex(i,1),3) = goal(3); % Save where this optimal value comes from (initial point 110)
    end 

    while V(goal(2),2) > 10000

        size_vertex = size(vertex);

        for i=1:size_vertex(1)
            finding=ismember(data(:,2),vertex(i,1));
            vertex_index2=find(finding);
            size_vertex_index2 = size(vertex_index2);
            for j=1:size_vertex_index2(1)
                vertex2(i,j,1) = data(vertex_index2(j),1);
                vertex2(i,j,2) = data(vertex_index2(j),3) + vertex(i,2);
                if V(vertex2(i,j,1),2) > vertex2(i,j,2) % Save Optimal values ('V')
                    V(vertex2(i,j,1),2) = vertex2(i,j,2); 
                    V(vertex2(i,j,1),3) = vertex(i,1); % Save where this optimal value comes from
                end
            end
        end

        vertex = [];

        % Delete same points
        vertex=unique(vertex2(:,:,1)); 

        % Delete zero array
        delete_zero=ismember(vertex(:,1),0);
        zero_index=find(delete_zero);
        vertex(zero_index,:) = [];

        size_vertex = size(vertex);

        for i=1:size_vertex(1)
                vertex(i,2)=V(vertex(i,1),2);
        end
    %     V
    %     waitforbuttonpress;
    end


    print_order(1,1)=V(goal(2),1);
    print_order(2,1)=V(goal(2),2);
    next = V(goal(2),3);

    flag = 0;
    k=2;
    while flag == 0
        if(next==goal(3))
            flag = 1;
        end
        print_order(1,k)=V(next,1);
        print_order(2,k)=V(next,2);
        next = V(next,3);
       k=k+1; 
    end

    print_order(2,end) =0;
    print_order;

    if p==1
    T = table(print_order);
    writetable(T,'output1.txt');
    type output1.txt;
    else
    T = table(print_order);
    writetable(T,'output2.txt');
    type output2.txt;
    end
    
end
    