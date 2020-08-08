function collision = maze_Collision_obstacle(A,B,obstacle1,obstacle2,obstacle3,obstacle4,obstacle5)
collision=[];
size_obstacle1=size(obstacle1);
size_obstacle2=size(obstacle2);
size_obstacle3=size(obstacle3);
size_obstacle4=size(obstacle4);
size_obstacle5=size(obstacle5);

num_intersec=1;

    for i=1:2:size_obstacle1(2)-3
        C = [obstacle1(i), obstacle1(i+1)];
        D = [obstacle1(i+2), obstacle1(i+3)];
        intersec = maze_Collision_check(A,B,C,D);
            if isempty(intersec)==false
                collision(num_intersec,:) = intersec;
                num_intersec = num_intersec + 1;
            end
    end
    
    for i=1:2:size_obstacle2(2)-3
        C = [obstacle2(i), obstacle2(i+1)];
        D = [obstacle2(i+2), obstacle2(i+3)];
        intersec = maze_Collision_check(A,B,C,D);
            if isempty(intersec)==false
                collision(num_intersec,:) = intersec;
                num_intersec = num_intersec + 1;
            end
    end
    
        for i=1:2:size_obstacle3(2)-3
        C = [obstacle3(i), obstacle3(i+1)];
        D = [obstacle3(i+2), obstacle3(i+3)];
        intersec = maze_Collision_check(A,B,C,D);
            if isempty(intersec)==false
                collision(num_intersec,:) = intersec;
                num_intersec = num_intersec + 1;
            end
        end
    
        for i=1:2:size_obstacle4(2)-3
        C = [obstacle4(i), obstacle4(i+1)];
        D = [obstacle4(i+2), obstacle4(i+3)];
        intersec = maze_Collision_check(A,B,C,D);
            if isempty(intersec)==false
                collision(num_intersec,:) = intersec;
                num_intersec = num_intersec + 1;
            end
        end
    
        for i=1:2:size_obstacle5(2)-3
        C = [obstacle5(i), obstacle5(i+1)];
        D = [obstacle5(i+2), obstacle5(i+3)];
        intersec = maze_Collision_check(A,B,C,D);
            if isempty(intersec)==false
                collision(num_intersec,:) = intersec;
                num_intersec = num_intersec + 1;
            end
        end
        

end