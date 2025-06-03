% Parameters
a2 = 1; % meters
theta1 = deg2rad(30);
theta2 = deg2rad(45);
d3 = 0.5;

% Rotation matrices
Rz1 = [cos(theta1), -sin(theta1), 0;
       sin(theta1), cos(theta1), 0;
       0, 0, 1];

Rz2 = [cos(theta2), -sin(theta2), 0;
       sin(theta2), cos(theta2), 0;
       0, 0, 1];

% Position vectors
p0 = [0;0;0];
p1 = p0; % after T1 translation is zero
p2 = p1 + Rz1 * [a2; 0; 0]; % Link 2 translation
p3 = p2 + Rz1 * Rz2 * [0; 0; d3]; % End-effector position

% Joint axes
z0 = [0;0;1];
z1 = Rz1 * [0;0;1];
z2 = Rz1 * Rz2 * [0;0;1];

% Jacobian columns
J1 = [cross(z0, p3 - p0); z0];
J2 = [cross(z1, p3 - p1); z1];
J3 = [z2; [0;0;0]];

% Assemble Jacobian
J = [J1, J2, J3];

% Display Jacobian
disp('Jacobian matrix J:');
disp(J);

% Plot manipulator
figure;
hold on; grid on; axis equal;
plot3([p0(1) p1(1)], [p0(2) p1(2)], [p0(3) p1(3)], 'r-', 'LineWidth', 3);
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'g-', 'LineWidth', 3);
plot3([p2(1) p3(1)], [p2(2) p3(2)], [p2(3) p3(3)], 'b-', 'LineWidth', 3);
scatter3(p3(1), p3(2), p3(3), 100, 'k', 'filled');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3-DOF RRP Manipulator');
view(135,30);
hold off;
