% Which results to plot.
pipe_name = 'crawlerTopX';

% Get the correct scale factor.
if strncmp(pipe_name, 'crawlerTop', 10)
    scale_factor = -5;
else 
    scale_factor = -0.1278;
end
% Get the camera focal length.
camera_f = 510;

% Setup the figure.
figure;
hold on;

% Plot the appropriate ground truth measures.
if strncmp(pipe_name, 'crawlerTop', 10)
    load([pipe_name '.mat']);

    plot(pose(:, 3)-pose(1, 3), pose(:, 1)-pose(1, 1), 'k', 'LineWidth', 2);
else
    load([pipe_name '_groundtruth.mat']);

    plot(camera_pos(:, 2), camera_pos(:, 1), 'k', 'LineWidth', 2);
end

% Plot the LKT results.
load([pipe_name '_lkt_results.mat']);

plot(scale_factor/camera_f*pos(:, 1), ...
     scale_factor/camera_f*pos(:, 2), ...
     'b', 'LineWidth', 2);

% Plot the LKT and tracker results.
load([pipe_name '_comb_results.mat']);

plot(scale_factor/camera_f*pos(:, 1), ...
     scale_factor/camera_f*pos(:, 2), ...
     'g', 'LineWidth', 2);

% Do some axis stuff.
limits = axis;

% Plot the marker position if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
    plot(marker_pos(:, 2) - marker_pos(1, 2), marker_pos(:, 1) - marker_pos(1, 1), 'r');
    plot(marker_pos(:, 2) - marker_pos(1, 2), marker_pos(:, 1) - marker_pos(1, 1), 'r.');
end

title('Evaluation of LKT and Tracking Results');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Groundtruth', 'LKT', 'LKT with Tracking', 'AR Tag Estimate');

% Set the axis limits.
axis(limits + [-.1 .1 -.1 .1]); 
