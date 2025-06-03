syms theta1 theta2 l1 l2 real

x = l1*cos(theta1) + l2*cos(theta1+theta2);
y = l1*sin(theta1) + l2*sin(theta1+theta2);

J = jacobian([x; y], [theta1, theta2]);
disp('Jacobian for 2-DOF planar RR manipulator:');
disp(J);
