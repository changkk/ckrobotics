function [x_hat_min, Sigma_min] = EKF_propagate(x_hat_plus, Sigma_plus, u, R, dt)


x_hat_min = x_hat_plus + u * dt;
Sigma_min = Sigma_plus + R;


end
