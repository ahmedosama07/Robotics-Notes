%% RRT-based motion planning for UR5

% Load UR5 model
robot = importrobot('universalUR5.urdf');
robot.DataFormat = 'row';

% Define environment obstacle
obstacle = collisionBox(0.4, 0.4, 0.4);
obstacle.Pose = trvec2tform([0.6, 0, 0.2]);

% RRT planner setup
planner = manipulatorRRT(robot, {obstacle});
planner.MaxConnectionDistance = 0.2;
planner.ValidationDistance = 0.05;
planner.MaxIterations = 1000;

% Start and goal
startConfig = homeConfiguration(robot);
goalConfig = [pi/4, -pi/4, pi/3, -pi/4, pi/3, 0];

% Plan path
path = plan(planner, startConfig, goalConfig);

% Visualize result
figure;
show(robot, startConfig, 'PreservePlot', false);
hold on;
show(obstacle);

for i = 1:size(path,1)
    show(robot, path(i,:), 'PreservePlot', true, 'Frames', 'off');
    pause(0.1);
end
title('RRT Planned Path');


%% Generate initial trajectory from RRT path
qPath = path;  % Using previous RRT path

% Time vector
t = linspace(0, 5, size(qPath, 1));

% Spline interpolation
for i = 1:size(qPath,2)
    qSpline(i,:) = spline(t, qPath(:,i), linspace(0,5,100));
end

% Plot smoothed trajectories
figure;
plot(linspace(0,5,100), qSpline');
xlabel('Time (s)');
ylabel('Joint Position (rad)');
title('Smoothed Trajectory (Joint Space)');
legend('q1','q2','q3','q4','q5','q6');

% Animate smoothed trajectory
figure;
for i = 1:100
    show(robot, qSpline(:,i)', 'PreservePlot', false, 'Frames', 'off');
    hold on;
    show(obstacle);
    drawnow;
end
title('Smoothed Trajectory Animation');

%% Simulate 2D LiDAR Scan
angles = linspace(-pi/2, pi/2, 360); % 180 degree scan
ranges = 1.0 + 0.1*rand(1, 360);     % Simulated data

scan = lidarScan(ranges, angles);

% Plot LiDAR scan
figure;
plot(scan);
title('Simulated 2D LiDAR Scan');

%% Use scan to detect obstacles and adjust planner
% For real applications, this would update the obstacle map
% E.g., dynamic obstacles or SLAM-based mapping

