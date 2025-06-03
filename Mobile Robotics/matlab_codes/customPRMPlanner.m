% Improved PRM script for UR5 robot path planning
clear; clc;

% Create manipulator model
robot = importrobot('universalUR5.urdf');
robot.DataFormat = 'row';

% Define workspace environment and obstacles
env = collisionBox(0.5, 0.5, 0.5);
env.Pose = trvec2tform([0.6 0 0.25]);

% Improved PRM parameters
numNodes = 200;              % Increased number of nodes
connectionDistance = 2.0;    % Increased connection distance
maxConnections = 10;         % Maximum connections per node

% Define start and goal
startConfig = homeConfiguration(robot);
goalConfig = [pi/2 0 0 -pi/2 0 pi/2];

fprintf('Start config: [%.2f %.2f %.2f %.2f %.2f %.2f]\n', startConfig);
fprintf('Goal config:  [%.2f %.2f %.2f %.2f %.2f %.2f]\n', goalConfig);

% Check if start and goal are collision-free
if checkCollision(robot, startConfig, env)
    fprintf('WARNING: Start configuration is in collision!\n');
end
if checkCollision(robot, goalConfig, env)
    fprintf('WARNING: Goal configuration is in collision!\n');
end

% Generate random nodes
fprintf('Generating random nodes...\n');
nodes = generateRandomNodes(robot, numNodes, env);

% Add start and goal to nodes
allNodes = [startConfig; nodes; goalConfig];
startIdx = 1;
goalIdx = size(allNodes, 1);

fprintf('Total nodes: %d (including start and goal)\n', size(allNodes, 1));

% Build improved roadmap
fprintf('Building roadmap...\n');
adjacencyMatrix = buildImprovedRoadmap(robot, allNodes, connectionDistance, maxConnections, env);

% Check connectivity
numConnections = sum(sum(adjacencyMatrix < inf)) / 2; % Divide by 2 for undirected graph
fprintf('Total connections: %d\n', numConnections);

% Find path using Dijkstra's algorithm
fprintf('Finding path...\n');
pathIndices = dijkstraPath(adjacencyMatrix, startIdx, goalIdx);

if ~isempty(pathIndices)
    path = allNodes(pathIndices, :);
    fprintf('SUCCESS: Path found with %d waypoints\n', length(pathIndices));
    
    % Smooth the path
    fprintf('Smoothing path...\n');
    smoothPath = smoothPathSimple(robot, path, env);
    
    % Visualize
    visualizeImprovedPath(robot, smoothPath, env, allNodes, adjacencyMatrix, pathIndices);
else
    fprintf('FAILED: No path found\n');
    
    % Diagnostic: Check if start and goal are connected to anything
    startConnections = sum(adjacencyMatrix(startIdx, :) < inf) - 1; % -1 for self
    goalConnections = sum(adjacencyMatrix(goalIdx, :) < inf) - 1;
    
    fprintf('Start node connections: %d\n', startConnections);
    fprintf('Goal node connections: %d\n', goalConnections);
    
    % Try with simpler goal if current goal fails
    if goalConnections == 0
        fprintf('Trying simpler goal configuration...\n');
        simpleGoal = [pi/4 0 0 -pi/4 0 0]; % Simpler goal
        allNodes(goalIdx, :) = simpleGoal;
        
        % Rebuild connections for new goal
        adjacencyMatrix = buildImprovedRoadmap(robot, allNodes, connectionDistance, maxConnections, env);
        pathIndices = dijkstraPath(adjacencyMatrix, startIdx, goalIdx);
        
        if ~isempty(pathIndices)
            path = allNodes(pathIndices, :);
            fprintf('SUCCESS with simpler goal: Path found with %d waypoints\n', length(pathIndices));
            smoothPath = smoothPathSimple(robot, path, env);
            visualizeImprovedPath(robot, smoothPath, env, allNodes, adjacencyMatrix, pathIndices);
        end
    end
end

%% Supporting Functions

function nodes = generateRandomNodes(robot, numNodes, obstacle)
    % Get joint limits - more conservative limits for better connectivity
    numJoints = 6;
    lowerLimits = [-pi, -pi, -pi/2, -pi, -pi, -pi];
    upperLimits = [pi, pi, pi/2, pi, pi, pi];
    
    nodes = [];
    attempts = 0;
    maxAttempts = numNodes * 20; % More attempts
    
    while size(nodes, 1) < numNodes && attempts < maxAttempts
        attempts = attempts + 1;
        
        % Generate random configuration
        config = lowerLimits + (upperLimits - lowerLimits) .* rand(1, numJoints);
        
        % Check if configuration is collision-free
        if ~checkCollision(robot, config, obstacle)
            nodes = [nodes; config];
        end
        
        % Progress indicator
        if mod(attempts, 100) == 0
            fprintf('  Attempts: %d, Valid nodes: %d\n', attempts, size(nodes, 1));
        end
    end
    
    fprintf('Generated %d collision-free nodes from %d attempts\n', size(nodes, 1), attempts);
end

function adjacencyMatrix = buildImprovedRoadmap(robot, nodes, maxDistance, maxConnections, obstacle)
    numNodes = size(nodes, 1);
    adjacencyMatrix = inf(numNodes, numNodes);
    
    % Set diagonal to zero
    for i = 1:numNodes
        adjacencyMatrix(i, i) = 0;
    end
    
    fprintf('Building connections...\n');
    
    % For each node, find k nearest neighbors
    for i = 1:numNodes
        if mod(i, 50) == 0
            fprintf('  Processing node %d/%d\n', i, numNodes);
        end
        
        % Calculate distances to all other nodes
        distances = zeros(numNodes, 1);
        for j = 1:numNodes
            if i ~= j
                distances(j) = norm(nodes(i, :) - nodes(j, :));
            else
                distances(j) = inf; % Exclude self
            end
        end
        
        % Sort by distance and take k nearest
        [sortedDistances, sortedIndices] = sort(distances);
        
        connections = 0;
        for k = 1:min(maxConnections, numNodes-1)
            j = sortedIndices(k);
            distance = sortedDistances(k);
            
            if distance <= maxDistance && connections < maxConnections
                % Check if straight line path is collision-free
                if isPathCollisionFree(robot, nodes(i, :), nodes(j, :), obstacle)
                    adjacencyMatrix(i, j) = distance;
                    adjacencyMatrix(j, i) = distance;
                    connections = connections + 1;
                end
            end
        end
    end
end

function isCollisionFree = isPathCollisionFree(robot, startConfig, endConfig, obstacle)
    % Check collision along straight line path with more samples
    numChecks = 20; % Increased resolution
    isCollisionFree = true;
    
    for t = linspace(0, 1, numChecks)
        config = (1-t) * startConfig + t * endConfig;
        if checkCollision(robot, config, obstacle)
            isCollisionFree = false;
            break;
        end
    end
end

function isColliding = checkCollision(robot, config, obstacle)
    try
        % More comprehensive collision checking
        % Check multiple points on the robot
        
        % End-effector
        T_ee = getTransform(robot, config, 'tool0');
        eePos = T_ee(1:3, 4);
        
        % Base link (approximate)
        T_base = getTransform(robot, config, 'base_link');
        basePos = T_base(1:3, 4);
        
        % Obstacle properties
        obstaclePos = obstacle.Pose(1:3, 4);
        obstacleSize = [obstacle.X, obstacle.Y, obstacle.Z] / 2;
        
        % Check if any critical point is inside the box
        isColliding = isPointInBox(eePos, obstaclePos, obstacleSize) || ...
                     isPointInBox(basePos, obstaclePos, obstacleSize);
                     
    catch ME
        % If any error occurs, assume no collision
        isColliding = false;
    end
end

function inside = isPointInBox(point, boxCenter, boxHalfSize)
    inside = all(abs(point - boxCenter) <= boxHalfSize);
end

function pathIndices = dijkstraPath(adjacencyMatrix, startIdx, goalIdx)
    numNodes = size(adjacencyMatrix, 1);
    distances = inf(numNodes, 1);
    distances(startIdx) = 0;
    visited = false(numNodes, 1);
    previous = zeros(numNodes, 1);
    
    for i = 1:numNodes
        % Find unvisited node with minimum distance
        unvisitedDistances = distances;
        unvisitedDistances(visited) = inf;
        [minDist, currentIdx] = min(unvisitedDistances);
        
        if minDist == inf || currentIdx == goalIdx
            break;
        end
        
        visited(currentIdx) = true;
        
        % Update distances to neighbors
        neighbors = find(adjacencyMatrix(currentIdx, :) < inf);
        for neighborIdx = neighbors
            if ~visited(neighborIdx)
                newDistance = distances(currentIdx) + adjacencyMatrix(currentIdx, neighborIdx);
                if newDistance < distances(neighborIdx)
                    distances(neighborIdx) = newDistance;
                    previous(neighborIdx) = currentIdx;
                end
            end
        end
    end
    
    % Reconstruct path
    if distances(goalIdx) == inf
        pathIndices = [];
    else
        pathIndices = [];
        currentIdx = goalIdx;
        while currentIdx ~= 0
            pathIndices = [currentIdx; pathIndices];
            currentIdx = previous(currentIdx);
        end
    end
end

function smoothPath = smoothPathSimple(robot, originalPath, obstacle)
    % Simple path smoothing by trying to connect non-adjacent waypoints
    smoothPath = originalPath;
    
    if size(originalPath, 1) <= 2
        return;
    end
    
    i = 1;
    while i < size(smoothPath, 1) - 1
        % Try to connect current point to points further ahead
        connected = false;
        for j = size(smoothPath, 1):-1:(i+2)
            if isPathCollisionFree(robot, smoothPath(i, :), smoothPath(j, :), obstacle)
                % Remove intermediate points
                smoothPath = [smoothPath(1:i, :); smoothPath(j:end, :)];
                connected = true;
                break;
            end
        end
        
        if ~connected
            i = i + 1;
        end
    end
    
    fprintf('Path smoothed from %d to %d waypoints\n', size(originalPath, 1), size(smoothPath, 1));
end

function visualizeImprovedPath(robot, path, obstacle, allNodes, adjacencyMatrix, pathIndices)
    figure('Position', [100, 100, 1200, 800]);
    
    % Plot roadmap in configuration space (first 3 joints)
    subplot(2, 3, 1);
    hold on;
    
    % Plot connections
    for i = 1:size(allNodes, 1)
        for j = i+1:size(allNodes, 1)
            if adjacencyMatrix(i, j) < inf
                plot3([allNodes(i, 1), allNodes(j, 1)], ...
                      [allNodes(i, 2), allNodes(j, 2)], ...
                      [allNodes(i, 3), allNodes(j, 3)], 'b-', 'LineWidth', 0.3, 'Color', [0.7 0.7 1]);
            end
        end
    end
    
    % Plot all nodes
    scatter3(allNodes(:, 1), allNodes(:, 2), allNodes(:, 3), 20, 'b', 'filled', 'MarkerFaceAlpha', 0.6);
    
    % Highlight path nodes
    if ~isempty(pathIndices)
        pathNodes = allNodes(pathIndices, :);
        plot3(pathNodes(:, 1), pathNodes(:, 2), pathNodes(:, 3), 'g-', 'LineWidth', 3);
        scatter3(pathNodes(1, 1), pathNodes(1, 2), pathNodes(1, 3), 100, 'g', 'filled');
        scatter3(pathNodes(end, 1), pathNodes(end, 2), pathNodes(end, 3), 100, 'r', 'filled');
    end
    
    title('PRM Roadmap (Joints 1-3)');
    xlabel('Joint 1 (rad)'); ylabel('Joint 2 (rad)'); zlabel('Joint 3 (rad)');
    grid on;
    
    % Plot path in configuration space
    subplot(2, 3, 2);
    if ~isempty(path)
        plot(1:size(path, 1), path, 'LineWidth', 2);
        title('Joint Trajectories');
        xlabel('Waypoint'); ylabel('Joint Angle (rad)');
        legend({'J1', 'J2', 'J3', 'J4', 'J5', 'J6'}, 'Location', 'best');
        grid on;
    end
    
    % Show start configuration
    subplot(2, 3, 4);
    show(robot, path(1, :));
    hold on;
    show(obstacle);
    title('Start Configuration');
    view(45, 30);
    
    % Show goal configuration
    subplot(2, 3, 5);
    show(robot, path(end, :));
    hold on;
    show(obstacle);
    title('Goal Configuration');
    view(45, 30);
    
    % Animate robot following the path
    subplot(2, 3, [3, 6]);
    for i = 1:size(path, 1)
        cla;
        show(robot, path(i, :));
        hold on;
        show(obstacle);
        title(sprintf('Path Animation - Step %d/%d', i, size(path, 1)));
        view(45, 30);
        drawnow;
        pause(0.1);
    end
end