%% 1. Setup: Robot, Environment, ROS
clc; clear; close all;

% Load manipulator model
robot = loadrobot('kinovaGen3', 'DataFormat', 'row', 'Gravity', [0 0 -9.81]);

% Define joint limits from robot model
numJoints = numel(robot.homeConfiguration);
jointMinLimits = zeros(1, numJoints);
jointMaxLimits = zeros(1, numJoints);

for i = 1:numJoints
    jointLimits = robot.Bodies{i+1}.Joint.PositionLimits; % [min max]
    jointMinLimits(i) = jointLimits(1);
    jointMaxLimits(i) = jointLimits(2);
end

% Initialize ROS
try
    rosnode list;
    fprintf('ROS already active.\n');
catch
    rosinit;
    fprintf('ROS initialized.\n');
end

% Subscribe to Octomap binary
octoSub = rossubscriber('/octomap_binary', 'octomap_msgs/Octomap');
fprintf('Subscribed to /octomap_binary\n');

%% 2. RRT-based Initial Path Planning
startConfig = homeConfiguration(robot);
goalConfig = deg2rad([45, 30, -20, 10, 15, 0, 0]);

env = {}; % Placeholder for dynamic map
planner = manipulatorRRT(robot, env);
planner.MaxConnectionDistance = 0.1;

disp('Planning initial path using RRT...');
path = plan(planner, startConfig, goalConfig);
if iscell(path)
    trajInit = cell2mat(path')';
else
    trajInit = path;
end

%% 3. CHOMP-based Trajectory Optimization
lambda_smooth = 1.0;
lambda_obs = 100.0;
numIterations = 50;
dt = 0.05;
traj = trajInit;

N = size(traj,1);
D = diag(ones(N-1,1),1) - diag(ones(N,1),0);
A_smooth = D' * D;

disp('Optimizing trajectory with CHOMP...');
for iter = 1:numIterations
    % Get latest Octomap
    octoMsg = receive(octoSub, 1.0);
    if isempty(octoMsg)
        warning('No Octomap received. Retaining previous map.');
        continue;
    end
    
    map3D = readBinaryOccupancyMap3D(octoMsg);
    env{1} = collisionOccupancyMap3D(map3D);

    % Obstacle gradient placeholder (to be implemented)
    obsGrad = zeros(size(traj));

    % Smoothness gradient
    smoothGrad = 2 * (A_smooth * traj);

    % CHOMP update step
    traj = traj - 0.01 * (lambda_smooth * smoothGrad + lambda_obs * obsGrad);

    % Enforce joint limits
    traj = max(traj, cellfun(@(c) c.JointPosition, jointMinLimits)');
    traj = min(traj, cellfun(@(c) c.JointPosition, jointMaxLimits)');
end

%% 4. Dynamic Constraints Check
q = traj;
q_dot = diff(q) / dt;
q_ddot = diff(q_dot) / dt;

jointVelocityLimit = repmat(1.0, 1, numel(startConfig));     % rad/s
jointAccelLimit = repmat(5.0, 1, numel(startConfig));         % rad/s²

disp('🔷 Checking dynamic constraints...');
for i = 1:size(q_dot,2)
    if max(abs(q_dot(:,i))) > jointVelocityLimit(i)
        warning('⚠️ Velocity limit exceeded at joint %d', i);
    end
    if max(abs(q_ddot(:,i))) > jointAccelLimit(i)
        warning('⚠️ Acceleration limit exceeded at joint %d', i);
    end
end

%% 5. 3D Visualization of Robot Trajectory
figure('Name','3D Trajectory Execution');
show(robot, traj(1,:), 'PreservePlot', false, 'Frames', 'off');
hold on; grid on; view(3);
show(map3D);

for i = 1:size(traj,1)
    show(robot, traj(i,:), 'PreservePlot', true, 'Frames', 'off');
    pause(0.05);
end
title('CHOMP-Optimized Trajectory in Dynamic 3D Map');

%% 6. Plot Joint Position, Velocity, Acceleration
timeVec = (0:dt:(size(q,1)-1)*dt)';
figure('Name','Joint Trajectory Plots');

subplot(3,1,1); plot(timeVec, q); title('Joint Positions q(t)');
ylabel('Rad'); grid on;

subplot(3,1,2); plot(timeVec(1:end-1), q_dot); title('Joint Velocities q\_dot(t)');
ylabel('Rad/s'); grid on;

subplot(3,1,3); plot(timeVec(1:end-2), q_ddot); title('Joint Accelerations q\_ddot(t)');
ylabel('Rad/s²'); xlabel('Time (s)'); grid on;

disp('✅ CHOMP trajectory complete.');

%% 7. Utility Function: Octomap Reader
function map3D = readBinaryOccupancyMap3D(octoMsg)
    msgStruct = rosReadBinaryOccupancyMap3D(octoMsg);
    map3D = occupancyMap3D;
    setOccupancy(map3D, msgStruct.OccupiedPoints, 1);
end
