clear; clc; close all;

% --- Robot Parameters ---
L1 = 1.0; L2 = 0.8;
joint_limits = [deg2rad(-90), deg2rad(90);
                deg2rad(-90), deg2rad(90);
                0, 0.5];
max_vel = [deg2rad(45); deg2rad(45); 0.1];
max_acc = [deg2rad(90); deg2rad(90); 0.2];

% --- Obstacle ---
obs_center = [1.2; 0.2; 0.3];
obs_radius = 0.2;

% --- Sampling Parameters ---
num_samples = 500;
k_neighbors = 10;

% --- Start and Goal ---
start_q = [deg2rad(0); deg2rad(0); 0.1];
goal_q  = [deg2rad(45); deg2rad(30); 0.3];

% --- Sample random nodes ---
nodes = zeros(3, num_samples);
i = 1;
while i <= num_samples
    q_sample = zeros(3,1);
    for j=1:3
        q_sample(j) = (joint_limits(j,2) - joint_limits(j,1))*rand + joint_limits(j,1);
    end
    if checkCollisionFull(q_sample, L1, L2, obs_center, obs_radius)
        continue;
    end
    nodes(:,i) = q_sample;
    i = i + 1;
end

% Add start and goal to nodes
nodes = [start_q, nodes, goal_q];
num_nodes = size(nodes,2);

% Precompute end-effector positions for all nodes
ee_positions = zeros(3, num_nodes);
for i=1:num_nodes
    ee_positions(:,i) = FK_3DOF(nodes(:,i), L1, L2);
end

% --- Build adjacency matrix ---
adjacency = inf(num_nodes);
for i=1:num_nodes
    dists = vecnorm(nodes - nodes(:,i), 2, 1);
    [~, idx] = sort(dists);
    neighbors = idx(2:min(k_neighbors+1, num_nodes));
    
    for nb = neighbors
        if adjacency(i, nb) == inf
            if edgeFeasible(nodes(:,i), nodes(:,nb), L1, L2, obs_center, obs_radius, max_vel, max_acc)
                dist_val = jointDist(nodes(:,i), nodes(:,nb));
                adjacency(i, nb) = dist_val;
                adjacency(nb, i) = dist_val;
            end
        end
    end
end

% --- A* Search ---
start_idx = 1;
goal_idx = num_nodes;
[dist, path_idx] = AStarSearch(adjacency, start_idx, goal_idx, ee_positions);

if isempty(path_idx)
    error('No feasible path found by A*');
end

path = nodes(:, path_idx);

% --- Visualize roadmap ---
figure; hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('PRM with A* Path Planning');

% Plot obstacle
[obs_x, obs_y, obs_z] = sphere(30);
surf(obs_center(1)+obs_radius*obs_x, obs_center(2)+obs_radius*obs_y, obs_center(3)+obs_radius*obs_z,...
    'FaceAlpha',0.3,'EdgeColor','none','FaceColor','r');

% Plot nodes
for i=1:num_nodes
    plot3(ee_positions(1,i), ee_positions(2,i), ee_positions(3,i), '.b', 'MarkerSize', 8);
end

% Plot edges
for i=1:num_nodes
    for j=i+1:num_nodes
        if adjacency(i,j) < inf
            plot3([ee_positions(1,i), ee_positions(1,j)], ...
                  [ee_positions(2,i), ee_positions(2,j)], ...
                  [ee_positions(3,i), ee_positions(3,j)], 'c-', 'LineWidth', 0.5);
        end
    end
end

% Plot path
path_ee = zeros(3, length(path_idx));
for i=1:length(path_idx)
    path_ee(:,i) = ee_positions(:,path_idx(i));
end
plot3(path_ee(1,:), path_ee(2,:), path_ee(3,:), 'ro-', 'LineWidth', 2, 'MarkerSize', 8);

legend('Obstacle', 'Nodes', 'Edges', 'A* Path');

% --- Animate path ---
figure; hold on; grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Manipulator Motion Along A* Path');

% Create obstacle
[obs_x, obs_y, obs_z] = sphere(20);
surf(obs_center(1)+obs_radius*obs_x, ...
     obs_center(2)+obs_radius*obs_y, ...
     obs_center(3)+obs_radius*obs_z, ...
     'FaceAlpha',0.3,'EdgeColor','none','FaceColor','r');

% Set axis limits
axis_lim = [-0.5 2 -1 1 -0.1 1];
axis(axis_lim);
view(45,30);

% Precompute all FK points for animation
all_points = cell(1, length(path_idx));
for i=1:length(path_idx)
    all_points{i} = FK_points(path(:,i), L1, L2, 10);
end

% Create initial plot
h_robot = plot3(all_points{1}(1,:), all_points{1}(2,:), all_points{1}(3,:), ...
             'b-', 'LineWidth', 2);
h_base = plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% Animation loop
for i=1:length(path_idx)
    set(h_robot, 'XData', all_points{i}(1,:), ...
                 'YData', all_points{i}(2,:), ...
                 'ZData', all_points{i}(3,:));
    
    base_point = all_points{i}(:,1);
    set(h_base, 'XData', base_point(1), ...
                'YData', base_point(2), ...
                'ZData', base_point(3));
    
    drawnow;
    pause(0.1);
end

% ==================== FUNCTION DEFINITIONS ====================
function [points] = FK_points(q, L1, L2, num_pts_per_link)
    theta1 = q(1); theta2 = q(2); d3 = q(3);
    base = [0;0;0];
    joint1 = [L1*cos(theta1); L1*sin(theta1); 0];
    joint2 = joint1 + [L2*cos(theta1 + theta2); L2*sin(theta1 + theta2); 0];
    ee = joint2 + [0;0;d3];
    
    points = [];
    for i=1:num_pts_per_link
        alpha = (i-1)/(num_pts_per_link-1);
        points = [points, base + alpha*(joint1 - base)];         % Link 1
        points = [points, joint1 + alpha*(joint2 - joint1)];     % Link 2
        points = [points, joint2 + alpha*(ee - joint2)];         % Link 3
    end
end

function collision = checkCollisionFull(q, L1, L2, obs_center, obs_radius)
    pts = FK_points(q, L1, L2, 10);
    collision = any(vecnorm(pts - obs_center, 2, 1) <= obs_radius);
end

function d = jointDist(q1, q2)
    d = norm(q1 - q2);
end

function feasible = edgeFeasible(q1, q2, L1, L2, obs_center, obs_radius, max_vel, max_acc)
    steps = 10;
    feasible = true;
    
    % Collision checking
    for i=0:steps
        s = i/steps;
        q_interp = q1 + s*(q2 - q1);
        if checkCollisionFull(q_interp, L1, L2, obs_center, obs_radius)
            feasible = false;
            return;
        end
    end
    
    % Dynamic constraints
    delta_q = abs(q2 - q1);
    time_per_joint = delta_q ./ max_vel;
    t_total = max(time_per_joint);
    
    avg_vel = delta_q / t_total;
    avg_acc = avg_vel / t_total;
    
    if any(avg_vel > max_vel) || any(avg_acc > max_acc)
        feasible = false;
    end
end

function pos = FK_3DOF(q, L1, L2)
    theta1 = q(1); theta2 = q(2); d3 = q(3);
    p1 = [L1*cos(theta1); L1*sin(theta1); 0];
    p2 = p1 + [L2*cos(theta1 + theta2); L2*sin(theta1 + theta2); 0];
    pos = p2 + [0;0;d3];
end

function [dist, path] = AStarSearch(adjacency, start_idx, goal_idx, ee_positions)
    num_nodes = size(adjacency,1);
    
    % Initialize data structures
    open_set = false(1, num_nodes);
    open_set(start_idx) = true;
    came_from = zeros(1, num_nodes);
    
    g_score = inf(1, num_nodes);
    g_score(start_idx) = 0;
    
    % Heuristic: Euclidean distance in task space
    h_score = vecnorm(ee_positions - ee_positions(:,goal_idx), 2, 1);
    
    f_score = inf(1, num_nodes);
    f_score(start_idx) = h_score(start_idx);
    
    while any(open_set)
        % Find node with minimum f_score in open set
        current = find(open_set);
        [~, idx] = min(f_score(current));
        current = current(idx);
        
        if current == goal_idx
            % Reconstruct path
            path = current;
            while came_from(current) ~= 0
                current = came_from(current);
                path = [current path];
            end
            dist = g_score(goal_idx);
            return;
        end
        
        open_set(current) = false;
        
        neighbors = find(adjacency(current,:) < inf);
        for neighbor = neighbors
            tentative_g = g_score(current) + adjacency(current, neighbor);
            
            if tentative_g < g_score(neighbor)
                came_from(neighbor) = current;
                g_score(neighbor) = tentative_g;
                f_score(neighbor) = g_score(neighbor) + h_score(neighbor);
                open_set(neighbor) = true;
            end
        end
    end
    
    % No path found
    dist = inf;
    path = [];
end