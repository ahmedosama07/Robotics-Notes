% Example joint trajectory planning for d3
q0 = 0.2; % initial d3
qf = 0.8; % final d3
tf = 2;   % time duration (s)

% Time vector
t = linspace(0, tf, 100);

% Coefficients
a0 = q0;
a1 = 0;
a2 = (3/(tf^2))*(qf - q0);
a3 = -(2/(tf^3))*(qf - q0);

% Position trajectory
q = a0 + a1*t + a2*t.^2 + a3*t.^3;

% Velocity trajectory
dq = a1 + 2*a2*t + 3*a3*t.^2;

% Plot
figure; subplot(2,1,1); plot(t, q, 'b', 'LineWidth', 2); title('Joint Position');
subplot(2,1,2); plot(t, dq, 'r', 'LineWidth', 2); title('Joint Velocity');
xlabel('Time (s)');
