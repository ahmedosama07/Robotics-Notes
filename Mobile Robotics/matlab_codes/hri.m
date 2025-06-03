clear; clc; close all;

% Parameters
dt = 0.01;            % Time step
T = 10;               % Total simulation time (seconds)
time = 0:dt:T;

% Desired trajectory (circular path in task space)
radius = 0.3;
omega = 0.2; % rad/s

% Robot parameters (simple 2-link planar arm)
l1 = 0.5; l2 = 0.4;  % link lengths (meters)

% Initial joint angles
q = [0; 0];          % rad
q_dot = [0; 0];

% Impedance control parameters (task-space mass, damping, stiffness)
Md = diag([2 2]);    % Desired inertia matrix (kg)
Bd = diag([20 20]);  % Desired damping matrix (Ns/m)
Kd = diag([100 100]);% Desired stiffness matrix (N/m)

% Joint limits (radians)
q_min = [-pi/2; -pi/2];
q_max = [pi/2; pi/2];

% Human-applied external force at end-effector (simulate interaction)
F_ext = @(t) [5*sin(2*pi*0.5*t); 2*cos(2*pi*0.5*t)]; % N

% Storage for plotting
q_store = zeros(2, length(time));
x_store = zeros(2, length(time));
Fstore = zeros(2, length(time));

for i = 1:length(time)
    t = time(i);
    
    % Desired end-effector position (circular trajectory)
    xd = radius * [cos(omega*t); sin(omega*t)] + [0.6; 0.3];
    xd_dot = radius * omega * [-sin(omega*t); cos(omega*t)];
    xd_ddot = radius * omega^2 * [-cos(omega*t); -sin(omega*t)];
    
    % Forward kinematics
    x = forward_kinematics(q, l1, l2);
    
    % Jacobian
    J = jacobian_2dof(q, l1, l2);
    
    % End-effector velocity
    x_dot = J * q_dot;
    
    % External force (human interaction)
    Fh = F_ext(t);
    
    % Impedance control law in task space:
    % Md * (x_ddot_desired - x_ddot) + Bd * (x_dot_desired - x_dot) + Kd * (x_desired - x) = F_ext
    % Rearrange to get desired acceleration:
    x_ddot_desired = xd_ddot + Md \ (Fh - Bd*(x_dot - xd_dot) - Kd*(x - xd));
    
    % Inverse kinematics velocity control:
    q_ddot = pinv(J) * (x_ddot_desired - jacobian_dot(q, q_dot, l1, l2) * q_dot);
    
    % Simple Euler integration for joint states
    q_dot = q_dot + q_ddot * dt;
    q = q + q_dot * dt;
    
    % Enforce joint limits
    q = max(min(q, q_max), q_min);
    
    % Store data
    q_store(:,i) = q;
    x_store(:,i) = x;
    Fstore(:,i) = Fh;
end

% Plot joint angles over time
figure;
subplot(3,1,1);
plot(time, q_store(1,:), 'LineWidth', 1.5);
hold on;
plot(time, q_store(2,:), 'LineWidth', 1.5);
title('Joint Angles \theta(t)');
ylabel('Angle (rad)');
legend('\theta_1', '\theta_2');
grid on;

% Joint velocities (numerical derivative)
q_dot_store = [diff(q_store(1,:))/dt 0; diff(q_store(2,:))/dt 0];
subplot(3,1,2);
plot(time, q_dot_store(1,:), 'LineWidth', 1.5);
hold on;
plot(time, q_dot_store(2,:), 'LineWidth', 1.5);
title('Joint Velocities \thetȧ(t)');
ylabel('Velocity (rad/s)');
legend('\thetȧ_1', '\thetȧ_2');
grid on;

% Joint accelerations (numerical derivative)
q_ddot_store = [diff(q_dot_store(1,:))/dt 0; diff(q_dot_store(2,:))/dt 0];
subplot(3,1,3);
plot(time, q_ddot_store(1,:), 'LineWidth', 1.5);
hold on;
plot(time, q_ddot_store(2,:), 'LineWidth', 1.5);
title('Joint Accelerations \thetä(t)');
ylabel('Acceleration (rad/s^2)');
xlabel('Time (s)');
legend('\thetä_1', '\thetä_2');
grid on;

% Animation of manipulator and human force
figure;
for i = 1:10:length(time)
    clf;
    % Robot links
    p0 = [0;0];
    p1 = [l1*cos(q_store(1,i)); l1*sin(q_store(1,i))];
    p2 = p1 + [l2*cos(sum(q_store(:,i))); l2*sin(sum(q_store(:,i)))];
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'b-', 'LineWidth', 3); hold on;
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'r-', 'LineWidth', 3);
    plot(p2(1), p2(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Plot human force vector at end-effector
    quiver(p2(1), p2(2), 0.05*Fstore(1,i), 0.05*Fstore(2,i), 'g', 'LineWidth', 2, 'MaxHeadSize', 2);
    
    axis equal;
    xlim([-0.2 1]);
    ylim([-0.2 1]);
    title(sprintf('Impedance Control HRI at t = %.2f s', time(i)));
    xlabel('X (m)');
    ylabel('Y (m)');
    grid on;
    drawnow;
end


%% --- Helper functions ---

function x = forward_kinematics(q, l1, l2)
    x = [l1*cos(q(1)) + l2*cos(q(1)+q(2));
         l1*sin(q(1)) + l2*sin(q(1)+q(2))];
end

function J = jacobian_2dof(q, l1, l2)
    J = [-l1*sin(q(1)) - l2*sin(q(1)+q(2)), -l2*sin(q(1)+q(2));
          l1*cos(q(1)) + l2*cos(q(1)+q(2)),  l2*cos(q(1)+q(2))];
end

function J_dot = jacobian_dot(q, q_dot, l1, l2)
    % Time derivative of the Jacobian
    q1 = q(1); q2 = q(2);
    q1_dot = q_dot(1); q2_dot = q_dot(2);
    
    J_dot = [-l1*cos(q1)*q1_dot - l2*cos(q1+q2)*(q1_dot + q2_dot), -l2*cos(q1+q2)*(q1_dot + q2_dot);
             -l1*sin(q1)*q1_dot - l2*sin(q1+q2)*(q1_dot + q2_dot), -l2*sin(q1+q2)*(q1_dot + q2_dot)];
end
