%% Parameters
m1 = 1.0; % kg
m2 = 1.0; % kg
l1 = 1.0; % m
l2 = 1.0; % m
I1 = 0.05; % kg.m^2
I2 = 0.05; % kg.m^2
g = 9.81; % m/s^2

%% Initial conditions
q0 = [0.1; 0.1]; % initial joint angles (rad)
dq0 = [0; 0];    % initial joint velocities (rad/s)

%% Time span
tspan = [0 10]; % simulate for 10 seconds

%% Pack parameters
params = [m1 m2 l1 l2 I1 I2 g];

%% ODE solver with adjusted tolerances
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t, y] = ode15s(@(t, y) dynamics(t, y, params), tspan, [q0; dq0], options);

%% Compute accelerations using dynamics function
ddq = zeros(length(t), 2);
for i = 1:length(t)
    dydt_i = dynamics(t(i), y(i,:)', params);
    ddq(i,:) = dydt_i(3:4)';
end

%% Plot results
figure;
subplot(3,1,1); plot(t, y(:,1), 'b', t, y(:,2), 'r'); 
xlabel('Time [s]'); ylabel('\theta (rad)');
legend('\theta_1','\theta_2'); title('Joint Positions');

subplot(3,1,2); plot(t, y(:,3), 'b', t, y(:,4), 'r');
xlabel('Time [s]'); ylabel('d\theta/dt (rad/s)');
legend('\theta_1','\theta_2'); title('Joint Velocities');

subplot(3,1,3); plot(t, ddq(:,1), 'b', t, ddq(:,2), 'r');
xlabel('Time [s]'); ylabel('d^2\theta/dt^2 (rad/s^2)');
legend('\theta_1','\theta_2'); title('Joint Accelerations');

%% Corrected dynamics function
function dydt = dynamics(~, y, params)
    % States
    q1 = y(1); q2 = y(2);
    dq1 = y(3); dq2 = y(4);

    % Parameters
    m1 = params(1); m2 = params(2); l1 = params(3); l2 = params(4);
    I1 = params(5); I2 = params(6); g = params(7);
    
    % Center of mass positions
    lc1 = l1/2;
    lc2 = l2/2;
    
    % Inertia matrix (corrected with lc1, lc2)
    M11 = m1*lc1^2 + I1 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I2;
    M12 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
    M21 = M12;
    M22 = m2*lc2^2 + I2;
    M = [M11, M12; M21, M22];
    
    % Coriolis & Centrifugal (scaled by lc2)
    C1 = -m2*l1*lc2*sin(q2)*(2*dq1*dq2 + dq2^2);
    C2 = m2*l1*lc2*sin(q2)*dq1^2;
    C = [C1; C2];
    
    % Gravity (using sin)
    G1 = (m1*lc1 + m2*l1)*g*sin(q1) + m2*lc2*g*sin(q1+q2);
    G2 = m2*lc2*g*sin(q1+q2);
    G = [G1; G2];
    
    % No external torque
    tau = [0; 0];
    
    % Joint accelerations
    ddq = M \ (tau - C - G);
    dydt = [dq1; dq2; ddq(1); ddq(2)];
end