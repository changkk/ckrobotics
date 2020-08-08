function intersec = triangle_Collision_check(A,B,C,D)

tol = eps(100000);

    if ((A(1)-B(1))*(C(2)-D(2))-(A(2)-B(2))*(C(1)-D(1)))==0
        intersec = [];
    else
        intersec2 = triangle_intersection(A,B,C,D);
         if (intersec2(1) > min(A(1),B(1))|| abs(intersec2(1) - min(A(1),B(1))) < tol) ...
                 && (intersec2(1)<max(A(1),B(1)) || abs(intersec2(1) - max(A(1),B(1))) < tol) ...
                 && (intersec2(2)>min(A(2),B(2)) || abs(intersec2(2) - min(A(2),B(2))) < tol) ...
                 && (intersec2(2)<max(A(2),B(2)) || abs(intersec2(2) - max(A(2),B(2))) < tol) ...
                 && (intersec2(1)>min(C(1),D(1)) || abs(intersec2(1) - min(C(1),D(1))) < tol) ...
                 && (intersec2(1)<max(C(1),D(1)) || abs(intersec2(1) - max(C(1),D(1))) < tol)...
                 && (intersec2(2)>min(C(2),D(2)) || abs(intersec2(2) - min(C(2),D(2))) < tol)...
                 && (intersec2(2)<max(C(2),D(2)) || abs(intersec2(2) - max(C(2),D(2))) < tol)
            intersec = intersec2;
%          elseif abs(intersec2(1)-A(1)) < tol && abs(intersec2(2)-A(2)) < tol || abs(intersec2(1)-B(1)) < tol && abs(intersec2(2)-B(2)) < tol || abs(intersec2(1)-C(1)) < tol && abs(intersec2(2)-C(2)) < tol || abs(intersec2(1)-D(1)) < tol && abs(intersec2(2)-D(2)) < tol 
%             intersec = intersec2;
         else
            intersec = [];
         end
    end
    
end