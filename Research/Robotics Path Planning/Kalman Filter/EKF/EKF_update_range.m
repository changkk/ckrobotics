function [x_hat_plus, Sigma_plus] = EKF_update_range(x_hat_min, Sigma_min, m, z_r, sigma_r)

    C=zeros(10,2);

    for i= 1:10
    C(i,1)=-(2*m(1,i) - 2*x_hat_min(1))/(2*((m(1,i) - x_hat_min(1))^2 + (m(2,i) - x_hat_min(2))^2)^(1/2));
    C(i,2)=-(2*m(2,i) - 2*x_hat_min(2))/(2*((m(1,i) - x_hat_min(1))^2 + (m(2,i) - x_hat_min(2))^2)^(1/2));
    end

    for j=1:10
        estimated_range(j,1) = sqrt((x_hat_min(1)-m(1,j))^2+(x_hat_min(2)-m(2,j))^2);
    end    
    
    
    K = Sigma_min * C' * inv(C*Sigma_min*C' + sigma_r^2*eye(10));
    x_hat_plus = x_hat_min + K * (z_r - estimated_range);
    Sigma_plus = (eye(2) - K*C) * Sigma_min;
    
end
