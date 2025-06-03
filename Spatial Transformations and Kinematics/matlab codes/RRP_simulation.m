function RRP_simulation(x, y, z, d1, d2_min, d2_max, d3_min, d3_max)
    % Inverse Kinematics
    theta1 = atan2(y, x);
    d3 = sqrt(x^2 + y^2);
    d2 = z - d1;
    
    % Joint limits check
    if d2 < d2_min || d2 > d2_max || d3 < d3_min || d3 > d3_max
        error('Target pose outside workspace!');
    end
    
    % Compute transformation matrices
    T1 = [cos(theta1), -sin(theta1), 0, 0;
          sin(theta1), cos(theta1), 0, 0;
          0, 0, 1, d1;
          0, 0, 0, 1];
      
    T2 = eye(4);
    T2(3,4) = d2;
    
    T3 = eye(4);
    T3(1,4) = d3;
    
    % Compute joint positions
    O0 = [0; 0; 0];
    O1 = T1(1:3,4);
    O2 = T1 * T2;
    O2 = O2(1:3,4);
    O3 = T1 * T2 * T3;
    O3 = O3(1:3,4);
    
    % Plot
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    plot3([O0(1), O1(1)], [O0(2), O1(2)], [O0(3), O1(3)], 'r', 'LineWidth', 3);
    plot3([O1(1), O2(1)], [O1(2), O2(2)], [O1(3), O2(3)], 'g', 'LineWidth', 3);
    plot3([O2(1), O3(1)], [O2(2), O3(2)], [O2(3), O3(3)], 'b', 'LineWidth', 3);
    plot3(O3(1), O3(2), O3(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    
    title('3-DOF RRP Manipulator');
    view(3);
end
