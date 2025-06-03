%% ROS Octomap Integration with Manipulator Path Planning
% This script demonstrates how to integrate Octomap data from ROS 
% with MATLAB's Robotics Toolbox for 3D path planning

%% 1. Initialize ROS Connection
try
    % Check if ROS is already initialized
    try
        % Try to check if ROS is already running by listing nodes
        rosnode list; 
        fprintf('ROS is already initialized.\n');
    catch
        % If it fails, initialize ROS
        rosinit('localhost'); % Or provide specific master URI
        fprintf('ROS connection initialized successfully.\n');
    end

catch ME
    error('Failed to initialize ROS: %s', ME.message);
end

%% 2. Create ROS Subscriber to Octomap
try
    % Create subscriber to octomap topic
    octoSub = rossubscriber('/octomap_binary', 'octomap_msgs/Octomap');
    fprintf('Octomap subscriber created.\n');
    
    % Wait for messages to be available
    fprintf('Waiting for Octomap messages...\n');
    
    % Get the latest Octomap message with timeout
    octoMsg = receive(octoSub, 10); % 10 second timeout
    fprintf('Octomap message received.\n');
    
catch ME
    error('Failed to receive Octomap message: %s\nEnsure octomap_server is running and publishing to /octomap_binary', ME.message);
end

%% 3. Convert Octomap to MATLAB Format
try
    % Convert ROS Octomap message to MATLAB occupancy map
    map3D = readBinaryOccupancyMap3D(octoMsg);
    fprintf('Octomap converted to MATLAB format.\n');
    
    % Display map properties
    fprintf('Map Resolution: %.3f m\n', map3D.Resolution);
    fprintf('Map Size: [%.1f, %.1f, %.1f] m\n', ...
        map3D.XWorldLimits(2) - map3D.XWorldLimits(1), ...
        map3D.YWorldLimits(2) - map3D.YWorldLimits(1), ...
        map3D.ZWorldLimits(2) - map3D.ZWorldLimits(1));
    
catch ME
    error('Failed to convert Octomap: %s', ME.message);
end

%% 4. Visualize 3D Occupancy Map
figure('Name', '3D Occupancy Map');
show(map3D);
title('3D Occupancy Map from Octomap');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on;
view(45, 30); % Set viewing angle

%% 5. Set up Robot Model (Example: 6-DOF manipulator)
% Create a sample robot - replace with your actual robot model
robot = rigidBodyTree('DataFormat', 'column');

% Add robot bodies and joints (example configuration)
% Replace this section with your actual robot URDF loading
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
joint2.JointAxis = [0 1 0];
joint2.HomePosition = 0;
body2.Joint = joint2;
addBody(robot, body2, 'link1');

% Add more links as needed for your robot...
% For demonstration, we'll use a simple 2-link robot

%% 6. Create Collision Environment
try
    % Create collision map from occupancy map
    collisionMap = collisionOccupancyMap3D(map3D);
    env = {collisionMap};
    fprintf('Collision environment created.\n');
    
catch ME
    warning('Failed to create collision environment: %s', ME.message);
    env = {}; % Empty environment as fallback
end

%% 7. Set up Path Planner
try
    % Create manipulator RRT planner
    planner = manipulatorRRT(robot, env);
    
    % Configure planner parameters
    planner.MaxConnectionDistance = 0.2;  % Maximum distance between nodes
    planner.ValidationDistance = 0.05;    % Step size for collision checking
    planner.MaxIterations = 5000;         % Maximum planning iterations
    planner.GoalReachedFcn = @(x,y) norm(x-y) < 0.1; % Goal tolerance
    
    fprintf('Path planner configured.\n');
    
catch ME
    error('Failed to create path planner: %s', ME.message);
end

%% 8. Define Start and Goal Configurations
% Home configuration (all joints at 0)
startConfig = homeConfiguration(robot);

% Goal configuration - adjust based on your robot's DOF
if robot.NumBodies >= 6
    goalConfig = [pi/4, -pi/4, pi/3, -pi/4, pi/3, 0]';
else
    % For simpler robot
    goalConfig = [pi/4, -pi/4]';
    goalConfig = goalConfig(1:robot.NumBodies);
end

fprintf('Start config: [%s]\n', num2str(startConfig', '%.2f '));
fprintf('Goal config: [%s]\n', num2str(goalConfig', '%.2f '));

%% 9. Plan Path
try
    fprintf('Planning path...\n');
    tic;
    path = plan(planner, startConfig, goalConfig);
    planTime = toc;
    
    if isempty(path)
        error('No valid path found!');
    end
    
    fprintf('Path planning completed in %.2f seconds.\n', planTime);
    fprintf('Path contains %d waypoints.\n', size(path, 1));
    
catch ME
    error('Path planning failed: %s', ME.message);
end

%% 10. Visualize Planned Path
figure('Name', 'RRT Path Planning with Octomap');

% Show start configuration
subplot(1,2,1);
show(robot, startConfig, 'PreservePlot', false);
hold on;
show(map3D, 'FastUpdate', true); % Fast update for better performance
title('Start Configuration with Obstacles');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);

% Show goal configuration
subplot(1,2,2);
show(robot, goalConfig, 'PreservePlot', false);
hold on;
show(map3D, 'FastUpdate', true);
title('Goal Configuration with Obstacles');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);

% Animate the planned path
figure('Name', 'Path Animation');
show(robot, startConfig, 'PreservePlot', false);
hold on;
show(map3D, 'FastUpdate', true);
title('RRT Path Planning with Octomap Obstacles');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);

% Animate path execution
fprintf('Animating path...\n');
for i = 1:size(path, 1)
    show(robot, path(i,:), 'PreservePlot', true, 'Frames', 'off', ...
         'Collisions', 'on');
    drawnow;
    pause(0.1);
end

%% 11. Path Analysis
% Calculate path length in joint space
pathLength = 0;
for i = 2:size(path, 1)
    pathLength = pathLength + norm(path(i,:) - path(i-1,:));
end

fprintf('\n=== Path Planning Results ===\n');
fprintf('Planning time: %.2f seconds\n', planTime);
fprintf('Number of waypoints: %d\n', size(path, 1));
fprintf('Path length (joint space): %.3f rad\n', pathLength);

%% 12. Optional: Save Results
% Uncomment to save results
% save('octomap_path_planning_results.mat', 'path', 'map3D', 'robot');
% fprintf('Results saved to octomap_path_planning_results.mat\n');

%% 13. Cleanup
rosshutdown;

fprintf('\nPath planning with Octomap integration completed successfully!\n');