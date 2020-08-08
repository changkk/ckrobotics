function [x_hat_plus, Sigma_plus] = GPS_update(x_hat_min, Sigma_min, z_g, sigma_g)

K = Sigma_min / (Sigma_min + sigma_g^2);
x_hat_plus = x_hat_min + K * (z_g - x_hat_min);
Sigma_plus = (1 - K) * Sigma_min;

end
