function val = triangle_inobstacle(alpha,obstacle)
    size_obstacle=size(obstacle);
    X = obstacle(1:2:size_obstacle(2));
    Y = obstacle(2:2:size_obstacle(2));
    val=inpolygon(alpha(1),alpha(2),X,Y);
end
