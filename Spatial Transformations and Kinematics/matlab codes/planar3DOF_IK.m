function [theta1, theta2, theta3] = planar3DOF_IK(l1, l2, l3, x, y, phi)
    % Compute wrist center
    x_c = x - l3 * cos(phi);
    y_c = y - l3 * sin(phi);
    r2 = x_c^2 + y_c^2;
    
    % theta2
    cos_theta2 = (r2 - l1^2 - l2^2) / (2 * l1 * l2);
    sin_theta2 = sqrt(1 - cos_theta2^2); % Elbow down
    theta2 = atan2(sin_theta2, cos_theta2);
    
    % theta1
    beta = atan2(y_c, x_c);
    gamma = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
    theta1 = beta - gamma;
    
    % theta3
    theta3 = phi - (theta1 + theta2);
end
