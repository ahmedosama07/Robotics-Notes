% Define a sample SDF (sphere)
[X, Y, Z] = meshgrid(-20:1:20, -20:1:20, -20:1:20);
sdf = sqrt(X.^2 + Y.^2 + Z.^2) - 10; % Sphere radius 10

map3D.Resolution = 1; % grid spacing

% Compute gradient of the SDF
[dx, dy, dz] = gradient(sdf, map3D.Resolution);

% Downsample factor
ds = 5;

% Video writer setup
videoFile = 'gradient_field_animation.mp4';
v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = 10;
open(v);

% Generate X, Y meshgrid for quiver2D
[X, Y] = meshgrid(1:size(sdf,2), 1:size(sdf,1));

% Animate through slices
figure('Color', 'w');
for k = 1:size(sdf,3)
    % Extract the current slice
    slice = sdf(:,:,k);
    neg_dx_slice = -dx(:,:,k);
    neg_dy_slice = -dy(:,:,k);

    % Plot signed distance field
    imagesc(slice);
    hold on;
    quiver(X(1:ds:end,1:ds:end), Y(1:ds:end,1:ds:end), ...
           neg_dx_slice(1:ds:end,1:ds:end), ...
           neg_dy_slice(1:ds:end,1:ds:end), ...
           'r'); % negative gradient quivers in red

    hold off;
    axis equal tight;
    title(['Negative Gradient Field Slice at Z = ', num2str(k)]);
    colorbar;
    xlabel('X');
    ylabel('Y');

    % Write current frame to video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

% Finalize video
close(v);

disp(['Video saved to: ', videoFile]);
