% Define symbolic variables
syms theta1 theta2 d3 real
a1 = 1; a2 = 1;

% Forward kinematics for SCARA
x = a1*cos(theta1) + a2*cos(theta1+theta2);
y = a1*sin(theta1) + a2*sin(theta1+theta2);
z = d3;

% Jacobian calculation
Jv = jacobian([x; y; z], [theta1, theta2, d3]);
disp('SCARA robot Jacobian linear velocity part:');
disp(Jv);
