t0 = 0; tf = 5;
theta0 = 0; thetaf = pi/2; % radians
dtheta0 = 0; dthetaf = 0;

A = [1 t0 t0^2 t0^3;
     0 1  2*t0 3*t0^2;
     1 tf tf^2 tf^3;
     0 1  2*tf 3*tf^2];

b = [theta0; dtheta0; thetaf; dthetaf];

a = A\b;

disp('Cubic polynomial coefficients:');
disp(a');
