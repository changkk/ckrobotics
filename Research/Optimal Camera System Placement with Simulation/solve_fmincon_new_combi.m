function [x,fval] = solve_fmincon_new_combi(x_initial)

global gamma target_num comb drone_num R t host_class wt ws f D ptz_ws ptz_resol ptz_f


x0 = [];
if(isempty(x_initial))
    for i = 1:drone_num
       x0 = [x0, 0, 0, pi/2]; 
    end
else
    x0 = [x_initial zeros(1,drone_num * 3  - size(x_initial,2))];
end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % gamma     (v: bnb)
    % [ h1h2(t1)v h2h1(t2)v h3h1(t3)v ]
    % [ h1h3(t1) h2h3(t2) h3h2(t3) ]
    % [ h1h4(t1) h2h4(t2) h3h4(t3) ]
    % [ h1h5(t1) h2h5(t2) h3h5(t3) ]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fun = @obj_func_with_gradient_comb;

%      = solve(f/range*wt*1440/ws*scale == 5, R);


% for i = 1:drone_num
%     fun = @(x) fun(x) - sqrt((x0(2*i-1)-x(2*i-1))^2+(x0(2*i)-x(2*i))^2);   
% end

%Inequality constraints. The should be in the form [A]{x}={b}
A=[];
b=[];
%Lower and upper bounds of variables
lb=[];
ub=[];
%Add empty matrices for coefficients of equality constraints and initial
%guess
Aeq=[];
beq=[];

%gs = GlobalSearch;
options = optimoptions('fmincon','Algorithm','interior-point','MaxFunctionEvaluations', 3000, 'MaxIterations', 10000);    
problem = createOptimProblem('fmincon', 'objective', fun, 'nonlcon', @constraints_new_combi, 'x0', x0, 'options', options);
[x,fval] = fmincon(problem);

