% PUMA 560 Inverse Kinematics (with custom transformation functions)
% Robot parameters
d1 = 0.6718;  % m
a1 = 0.4318;  % m
a2 = 0.0203;  % m
d4 = 0.15005; % m
d6 = 0.1;     % m

% === Custom transformation functions (replace Robotics Toolbox) ===
rotz = @(theta) [cos(theta) -sin(theta) 0; 
                 sin(theta)  cos(theta) 0; 
                 0           0          1];

rotx = @(alpha) [1  0          0; 
                 0  cos(alpha) -sin(alpha); 
                 0  sin(alpha)  cos(alpha)];

transl = @(x,y,z) [x; y; z];

% Homogeneous transformation matrix builders
T_rotz = @(theta) [rotz(theta) [0;0;0]; 0 0 0 1];
T_rotx = @(alpha) [rotx(alpha) [0;0;0]; 0 0 0 1];
T_transl = @(x,y,z) [eye(3) [x;y;z]; 0 0 0 1];

% === Reachable end-effector pose ===
p = [0.4605; 0.2659; 0.8836]; % Position (within workspace)
R = eye(3);                    % Orientation (identity)

% Wrist center calculation
p_w = p - d6 * R(:,3);

% Calculate theta1
theta1 = atan2(p_w(2), p_w(1));

% r and s calculations
r = sqrt(p_w(1)^2 + p_w(2)^2) - a1;
s = p_w(3) - d1;

% Cosine law for theta3
D = (r^2 + s^2 - a2^2 - d4^2) / (2 * a2 * d4);

% Check workspace reachability
if abs(D) > 1
    error('Position unreachable: |D| = %.4f > 1', abs(D));
end

% Calculate theta3 (elbow down configuration)
theta3 = atan2(sqrt(1 - D^2), D); 

% Calculate theta2
beta = atan2(s, r);
gamma = atan2(d4 * sin(theta3), a2 + d4 * cos(theta3));
theta2 = beta - gamma;

% === Forward kinematics using custom functions ===
% Link 1: T01
T01 = T_rotz(theta1) * T_transl(0, 0, d1) * T_rotx(-pi/2);

% Link 2: T12
T12 = T_rotz(theta2) * T_transl(a1, 0, 0);

% Link 3: T23
T23 = T_rotz(theta3) * T_transl(a2, 0, 0) * T_rotx(-pi/2);

% Compound transformation
T03 = T01 * T12 * T23;
R03 = T03(1:3, 1:3);

% Compute wrist rotation
R36 = R03' * R;

% Calculate wrist angles (theta4, theta5, theta6)
theta5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
theta4 = atan2(R36(2,3), R36(1,3));
theta6 = atan2(-R36(3,2), R36(3,1));

% Display results
fprintf('\nInverse Kinematics Solution (radians):\n');
disp([theta1, theta2, theta3, theta4, theta5, theta6]);

fprintf('\nInverse Kinematics Solution (degrees):\n');
disp(rad2deg([theta1, theta2, theta3, theta4, theta5, theta6]));