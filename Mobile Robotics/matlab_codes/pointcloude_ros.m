%% Point Cloud-Based Path Planning with ROS Integration
% This script receives a PointCloud2 message from ROS, converts it to a
% voxelized occupancy map, and uses it for motion planning with a manipulator.

%% 1. Initialize ROS Connection
try
    % Try using a simple ROS call to check if ROS is active
    rosnode list;
    fprintf('ROS is already initialized.\n');
catch
    rosinit('localhost'); % Replace with your ROS master if remote
    fprintf('ROS connection initialized successfully.\n');
end

%% 2. Subscribe to PointCloud2 Topic
try
    ptCloudSub = rossubscriber('/camera/depth/points', 'sensor_msgs/PointCloud2');
    fprintf('Subscribed to /camera/depth/points\n');
    
    fprintf('Waiting for PointCloud2 message...\n');
    ptCloudMsg = receive(ptCloudSub, 10); % 10 sec timeout
    fprintf('PointCloud2 message received.\n');
catch ME
    error('Failed to receive PointCloud2 message: %s\nEnsure the depth camera is publishing.', ME.message);
end

%% 3. Convert PointCloud to MATLAB Format
try
    ptCloud = rosReadXYZ(ptCloudMsg);
    ptCloud = pointCloud(ptCloud);
    fprintf('Point cloud converted to MATLAB format.\n');
catch ME
    error('Error processing PointCloud2 message: %s', ME.message);
end

%% 4. Visualize Point Cloud
figure('Name', 'Raw Point Cloud');
pcshow(ptCloud);
title('3D Point Cloud from Depth Camera');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
grid on; view(45, 30);

%% 5. Voxelize Point Cloud into Occupancy Map
try
    voxelSize = 0.02; % 2 cm resolution
    map3D = occupancyMap3D(voxelSize);
    insertPointCloud(map3D, [0, 0, 0], ptCloud.Location, 0.1); % 0.1m max range
    fprintf('Voxelized occupancy map created.\n');
catch ME
    error('Failed to voxelize point cloud: %s', ME.message);
end

%% 6. Visualize Voxel Map
figure('Name', 'Voxelized 3D Occupancy Map');
show(map3D);
title('Voxelized 3D Occupancy Map from Point Cloud');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30); grid on;

%% 7. Define Robot Model (Example)
% Use your robot model here or load URDF
robot = rigidBodyTree('DataFormat', 'column');

% Basic 2-link example (replace with actual robot)
body1 = rigidBody('link1');
joint1 = rigidBodyJoint('joint1', 'revolute');
joint1.JointAxis = [0 0 1];
body1.Joint = joint1;
addBody(robot, body1, 'base');

body2 = rigidBody('link2');
joint2 = rigidBodyJoint('joint2', 'revolute');
joint2.JointAxis = [0 1 0];
body2.Joint = joint2;
addBody(robot, body2, 'link1');

%% 8. Define Start and Goal Configurations
startConfig = homeConfiguration(robot);

if robot.NumBodies >= 6
    goalConfig = [pi/4, -pi/4, pi/3, -pi/4, pi/3, 0]';
else
    goalConfig = [pi/4, -pi/4]';
    goalConfig = goalConfig(1:robot.NumBodies);
end

fprintf('Start config: [%s]\n', num2str(startConfig', '%.2f '));
fprintf('Goal config:  [%s]\n', num2str(goalConfig', '%.2f '));

%% 9. Create Collision Environment
try
    collisionMap = collisionOccupancyMap3D(map3D);
    env = {collisionMap};
    fprintf('Collision environment created from voxel map.\n');
catch ME
    warning('Collision environment creation failed: %s', ME.message);
    env = {};
end

%% 10. Set Up Path Planner
planner = manipulatorRRT(robot, env);
planner.MaxConnectionDistance = 0.2;
planner.ValidationDistance = 0.05;
planner.MaxIterations = 3000;
planner.GoalReachedFcn = @(x, y) norm(x - y) < 0.1;

%% 11. Plan a Path
fprintf('Planning path...\n');
tic;
path = plan(planner, startConfig, goalConfig);
planTime = toc;

if isempty(path)
    error('No valid path found.');
end

fprintf('Path found in %.2f seconds with %d waypoints.\n', planTime, size(path,1));

%% 12. Visualize Planned Path
figure('Name', 'Path Planning with Point Cloud Obstacles');
show(robot, startConfig, 'PreservePlot', false);
hold on;
show(map3D);
title('Path Planning with 3D Point Cloud Obstacles');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(45, 30);

for i = 1:size(path, 1)
    show(robot, path(i,:), 'PreservePlot', true, 'Frames', 'off');
    pause(0.1);
end

%% 13. Path Analysis
pathLength = 0;
for i = 2:size(path, 1)
    pathLength = pathLength + norm(path(i,:) - path(i-1,:));
end

fprintf('\n=== Path Planning Results ===\n');
fprintf('Planning time: %.2f seconds\n', planTime);
fprintf('Waypoints: %d\n', size(path, 1));
fprintf('Joint-space path length: %.3f rad\n', pathLength);

%% 14. Shutdown ROS
rosshutdown;
fprintf('\nROS connection closed. Script completed successfully.\n');
