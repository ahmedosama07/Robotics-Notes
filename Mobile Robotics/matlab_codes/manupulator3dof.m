%% 3-DOF Planar Manipulator with PD Control
% Parameters
m1 = 1; m2 = 1; m3 = 0.5; 
l1 = 1; l2 = 1; l3 = 0.5; 
I1 = 0.05; I2 = 0.05; I3 = 0.02; 
g = 9.81;
lc1 = l1/2; lc2 = l2/2; lc3 = l3/2;

% Initial conditions
q0 = [0.1; 0.1; 0.1];
dq0 = [0; 0; 0];
tspan = [0 10];
params = [m1, m2, m3, l1, l2, l3, lc1, lc2, lc3, I1, I2, I3, g];

% PD Controller Parameters
Kp = diag([50, 50, 30]); % Proportional gains
Kd = diag([10, 10, 6]);  % Derivative gains

% Desired trajectory functions
qd_fun = @(t) [0.5*sin(0.2*t); 0.3*sin(0.2*t); 0.2*sin(0.2*t)];
dqd_fun = @(t) [0.5*0.2*cos(0.2*t); 0.3*0.2*cos(0.2*t); 0.2*0.2*cos(0.2*t)];

% ODE15s integration with controller
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
[t, y] = ode15s(@(t,y) dynamics3dof_controlled(t, y, params, qd_fun, dqd_fun, Kp, Kd),...
                tspan, [q0; dq0], options);

% Compute accelerations using dynamics function
ddq = zeros(length(t), 3);
for i = 1:length(t)
    dydt_i = dynamics3dof_controlled(t(i), y(i,:)', params, qd_fun, dqd_fun, Kp, Kd);
    ddq(i,:) = dydt_i(4:6)';
end

% Plot joint trajectories
figure('Name', '3-DOF Joint Trajectories');
subplot(3,1,1); 
plot(t, y(:,1), t, y(:,2), t, y(:,3));
xlabel('Time [s]'); ylabel('\theta (rad)'); title('Joint Positions');
legend('\theta_1','\theta_2','\theta_3');
grid on;

subplot(3,1,2); 
plot(t, y(:,4), t, y(:,5), t, y(:,6));
xlabel('Time [s]'); ylabel('d\theta/dt (rad/s)'); title('Joint Velocities');
legend('\theta_1','\theta_2','\theta_3');
grid on;

subplot(3,1,3); 
plot(t, ddq(:,1), t, ddq(:,2), t, ddq(:,3));
xlabel('Time [s]'); ylabel('d^2\theta/dt^2 (rad/s^2)'); title('Joint Accelerations');
legend('\theta_1','\theta_2','\theta_3');
grid on;

% 3D Animation
figure('Name', '3-DOF Manipulator Animation'); 
axis equal; grid on; view(3);
xlim([-2.5, 2.5]); ylim([-2.5, 2.5]); zlim([-0.5, 0.5]);
xlabel('X'); ylabel('Y'); zlabel('Z'); 
title('3-DOF Planar Manipulator Animation');

% Plot desired trajectory
t_desired = linspace(tspan(1), tspan(2), 100);
qd_desired = arrayfun(qd_fun, t_desired, 'UniformOutput', false);
qd_desired = cell2mat(qd_desired);
ee_desired = zeros(3, length(t_desired));
for i = 1:length(t_desired)
    q = qd_desired(:,i);
    ee_desired(:,i) = [l1*cos(q(1)) + l2*cos(q(1)+q(2)) + l3*cos(q(1)+q(2)+q(3)); 
                      l1*sin(q(1)) + l2*sin(q(1)+q(2)) + l3*sin(q(1)+q(2)+q(3)); 
                      0];
end
plot3(ee_desired(1,:), ee_desired(2,:), ee_desired(3,:), 'm--', 'LineWidth', 1.5);
hold on;

% Animation loop
for i = 1:20:length(t)
    q = y(i, 1:3);
    % Forward kinematics for 3 links (planar in XY)
    p0 = [0; 0; 0];
    p1 = [l1*cos(q(1)); l1*sin(q(1)); 0];
    p2 = [p1(1) + l2*cos(q(1)+q(2)); p1(2) + l2*sin(q(1)+q(2)); 0];
    p3 = [p2(1) + l3*cos(q(1)+q(2)+q(3)); p2(2) + l3*sin(q(1)+q(2)+q(3)); 0];
    
    % Plot links
    plot3([p0(1), p1(1)], [p0(2), p1(2)], [0, 0], 'b-', 'LineWidth', 2); 
    hold on;
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [0, 0], 'r-', 'LineWidth', 2);
    plot3([p2(1), p3(1)], [p2(2), p3(2)], [0, 0], 'g-', 'LineWidth', 2);
    
    % Plot joints
    plot3(p0(1), p0(2), 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(p1(1), p1(2), 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(p2(1), p2(2), 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot3(p3(1), p3(2), 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Plot end-effector trajectory
    plot3(p3(1), p3(2), 0, 'c.', 'MarkerSize', 8);
    
    hold off;
    drawnow;
end
legend('Desired Trajectory', 'Actual Trajectory', 'Location', 'best');

%% Symbolic Dynamic Model for 3-DOF Planar Manipulator
syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 real
syms l1 l2 l3 m1 m2 m3 I1 I2 I3 g real

% Position of centers of mass
r1 = [l1/2*cos(q1); l1/2*sin(q1); 0];
r2 = [l1*cos(q1) + l2/2*cos(q1+q2); 
      l1*sin(q1) + l2/2*sin(q1+q2); 
      0];
r3 = [l1*cos(q1) + l2*cos(q1+q2) + l3/2*cos(q1+q2+q3);
      l1*sin(q1) + l2*sin(q1+q2) + l3/2*sin(q1+q2+q3);
      0];

% Velocities
v1 = jacobian(r1, [q1, q2, q3]) * [dq1; dq2; dq3];
v2 = jacobian(r2, [q1, q2, q3]) * [dq1; dq2; dq3];
v3 = jacobian(r3, [q1, q2, q3]) * [dq1; dq2; dq3];

% Kinetic energy (including rotational)
T = 0.5*m1*(v1.'*v1) + 0.5*I1*dq1^2 + ...
    0.5*m2*(v2.'*v2) + 0.5*I2*(dq1 + dq2)^2 + ...
    0.5*m3*(v3.'*v3) + 0.5*I3*(dq1 + dq2 + dq3)^2;

% Potential energy
V = m1*g*r1(2) + m2*g*r2(2) + m3*g*r3(2);

% Lagrangian
L = T - V;

% Equations of motion
q = [q1; q2; q3];
dq = [dq1; dq2; dq3];
ddq = [ddq1; ddq2; ddq3];
tau = sym('tau', [3, 1]);

dL_dq = jacobian(L, q).';
dL_ddq = jacobian(L, dq).';
d_dt_dL_ddq = jacobian(dL_ddq, [q; dq]) * [dq; ddq];

EOM = d_dt_dL_ddq - dL_dq - tau;
EOM = simplify(EOM);

% Display equations
disp('Symbolic Equations of Motion for 3-DOF Planar Manipulator:');
for i = 1:3
    disp(['tau', num2str(i), ' = ', char(EOM(i))]);
end

%% 6-DOF Spatial Manipulator Visualization (UR5-like)
% DH Parameters for UR5
d1 = 0.0892; d4 = 0.1093; d5 = 0.0948; d6 = 0.0825;
a2 = 0.425; a3 = 0.392;

% Standard DH parameters [a, alpha, d, theta]
DH = [0    pi/2   d1   0;
      a2   0      0    0;
      a3   0      0    0;
      0    pi/2   d4   0;
      0   -pi/2   d5   0;
      0    0      d6   0];

% Transformation matrix function
T_transform = @(a, alpha, d, theta) ...
    [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0,          sin(alpha),             cos(alpha),            d;
     0,          0,                       0,                    1];

% Create visualization
figure('Name', '6-DOF UR5-like Manipulator');
hold on; axis equal; grid on; view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('6-DOF Spatial Manipulator Visualization');

% Define joint angles for a sample configuration
q_6dof = [0, -pi/2, pi/2, 0, pi/2, 0];

% Compute transformations
T_all = cell(1,7);
T_all{1} = eye(4);
for i = 1:size(DH,1)
    a = DH(i,1);
    alpha = DH(i,2);
    d = DH(i,3);
    theta = DH(i,4) + q_6dof(i);
    
    T_i = T_transform(a, alpha, d, theta);
    T_all{i+1} = T_all{i} * T_i;
end

% Extract positions
positions = zeros(3,7);
for i = 1:7
    positions(:,i) = T_all{i}(1:3,4);
end

% Plot links and joints
plot3(positions(1,:), positions(2,:), positions(3,:), 'o-', 'LineWidth', 2, 'MarkerFaceColor', 'k');
for i = 1:7
    text(positions(1,i), positions(2,i), positions(3,i), ['P', num2str(i-1)], ...
        'FontSize', 12, 'FontWeight', 'bold');
end

% Plot coordinate frames
colors = {'r', 'g', 'b'};
for i = 1:6
    R = T_all{i}(1:3,1:3);
    origin = T_all{i}(1:3,4);
    
    for j = 1:3
        axis_end = origin + 0.1 * R(:,j);
        plot3([origin(1), axis_end(1)], ...
              [origin(2), axis_end(2)], ...
              [origin(3), axis_end(3)], ...
              colors{j}, 'LineWidth', 1.5);
    end
end

% Set plot limits
all_pos = positions(:);
max_val = max(abs(all_pos)) + 0.1;
xlim([-max_val, max_val]);
ylim([-max_val, max_val]);
zlim([0, 1.5]);
view(45,30);
hold off;

% Example usage of interactive visualization:
% visualize_6dof([0, -pi/2, pi/2, 0, pi/2, 0], DH);
% visualize_6dof([pi/3, -pi/3, pi/3, pi/4, -pi/4, 0], DH);

%% Function Definitions (must be at the end of the script)

% Controlled dynamics function
function dydt = dynamics3dof_controlled(t, y, params, qd_fun, dqd_fun, Kp, Kd)
    % Unpack parameters
    m1 = params(1); m2 = params(2); m3 = params(3);
    l1 = params(4); l2 = params(5); l3 = params(6);
    lc1 = params(7); lc2 = params(8); lc3 = params(9);
    I1 = params(10); I2 = params(11); I3 = params(12);
    g = params(13);
    
    % States
    q = y(1:3);
    dq = y(4:6);
    q1 = q(1); q2 = q(2); q3 = q(3);
    dq1 = dq(1); dq2 = dq(2); dq3 = dq(3);
    
    % Inertia matrix
    a1 = I1 + m1*lc1^2 + m2*l1^2 + m3*l1^2;
    a2 = I2 + m2*lc2^2 + m3*l2^2;
    a3 = I3 + m3*lc3^2;
    b1 = m2*l1*lc2 + m3*l1*l2;
    b2 = m3*l1*lc3;
    b3 = m3*l2*lc3;
    
    M11 = a1 + a2 + a3 + 2*b1*cos(q2) + 2*b2*cos(q2+q3) + 2*b3*cos(q3);
    M12 = a2 + a3 + b1*cos(q2) + b2*cos(q2+q3) + 2*b3*cos(q3);
    M13 = a3 + b2*cos(q2+q3) + b3*cos(q3);
    M21 = M12;
    M22 = a2 + a3 + 2*b3*cos(q3);
    M23 = a3 + b3*cos(q3);
    M31 = M13;
    M32 = M23;
    M33 = a3;
    M = [M11, M12, M13; M21, M22, M23; M31, M32, M33];
    
    % Coriolis and centrifugal vector
    h1 = -b1*sin(q2)*(2*dq1*dq2 + dq2^2) ...
          - b2*sin(q2+q3)*(2*dq1*(dq2+dq3) + (dq2+dq3)^2) ...
          - b3*sin(q3)*(2*dq1*dq3 + 2*dq2*dq3 + dq3^2);
    h2 = b1*sin(q2)*dq1^2 ...
         - b3*sin(q3)*(2*dq2*dq3 + dq3^2) ...
         - b2*sin(q2+q3)*(dq1 + dq2 + dq3)^2;
    h3 = b2*sin(q2+q3)*dq1^2 + b3*sin(q3)*(dq1 + dq2)^2;
    h = [h1; h2; h3];
    
    % Gravity vector
    G1 = -g * ( (m1*lc1 + m2*l1 + m3*l1)*sin(q1) + ...
                (m2*lc2 + m3*l2)*sin(q1+q2) + ...
                m3*lc3*sin(q1+q2+q3) );
    G2 = -g * ( (m2*lc2 + m3*l2)*sin(q1+q2) + ...
                m3*lc3*sin(q1+q2+q3) );
    G3 = -g * ( m3*lc3*sin(q1+q2+q3) );
    G = [G1; G2; G3];
    
    % PD Control Law
    qd = qd_fun(t);
    dqd = dqd_fun(t);
    e = qd - q;
    de = dqd - dq;
    tau = Kp*e + Kd*de;
    
    % Solve for accelerations
    ddq = M \ (tau - h - G);
    dydt = [dq; ddq];
end

% Interactive visualization function
function visualize_6dof(q, DH)
    figure('Name', 'Interactive 6-DOF Visualization');
    hold on; axis equal; grid on; view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Interactive 6-DOF Manipulator Visualization');
    
    % Transformation matrix function
    T = @(a, alpha, d, theta) ...
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,          sin(alpha),             cos(alpha),            d;
         0,          0,                       0,                    1];
    
    % Compute transformations
    T_all = cell(1,7);
    T_all{1} = eye(4);
    for i = 1:size(DH,1)
        a = DH(i,1);
        alpha = DH(i,2);
        d = DH(i,3);
        theta = DH(i,4) + q(i);
        
        T_i = T(a, alpha, d, theta);
        T_all{i+1} = T_all{i} * T_i;
    end
    
    % Extract positions
    positions = zeros(3,7);
    for i = 1:7
        positions(:,i) = T_all{i}(1:3,4);
    end
    
    % Plot links and joints
    plot3(positions(1,:), positions(2,:), positions(3,:), 'o-', 'LineWidth', 2, 'MarkerFaceColor', 'k');
    
    % Set plot limits
    all_pos = positions(:);
    max_val = max(abs(all_pos)) + 0.1;
    xlim([-max_val, max_val]);
    ylim([-max_val, max_val]);
    zlim([0, 1.5]);
    view(45,30);
    hold off;
end