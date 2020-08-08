function [x_true, u, z_r] = rws_2D(N, dt, x_true_1, u_true, m, sigma_u, sigma_r)


% state dimension
n = length(x_true_1);

% preallocate
x_true = zeros(n,N);

% for each timestep
%   apply u_true to determine x_true
x_true(:,1) = x_true_1;

for i = 2 : N
    
    % x_k = x_k-1 + uk*dt
    x_true(:,i) = x_true(:,i-1) + u_true(:,i)*dt;
    
end

% Generate noisy u by sampling from Gaussian(mean u_true, std sigma_u)
% assume first u is always zero so that the robot starts at rest
u = u_true + sigma_u .* randn(n,N);
u(:,1) = 0;

% Generate noisy z_r by sampling from Gaussian(mean x_true, std sigma_g)
z_r = zeros(10,N);

for i=1:N
    for j=1:10
        z_r(j,i) = sqrt((x_true(1,i)-m(1,j))^2+(x_true(2,i)-m(2,j))^2) + sigma_r * randn(1);
    end    
end

end