% Which results to plot.
pipe_name = 'crawler2';

% Get the correct scale factor.
if strncmp(pipe_name, 'crawlerTop', 10)
    scale_factor = -5;
else 
    scale_factor = -0.1278;
end
% Get the camera focal length.
camera_f = 510;

% Get the appropriate ground truth measures.
if strncmp(pipe_name, 'crawlerTop', 10)
    load([pipe_name '.mat']);

    groundtruth = pose(:, [3 1]);
    groundtruth_angle = pose(:, 6);
else
    load([pipe_name '_groundtruth.mat']);

    groundtruth = camera_pos(:, [2, 1]);
    groundtruth_angle = -camera_pos(:, 3);
end

% Get the LKT results.
load([pipe_name '_lkt_results.mat']);

% Find nearest neighbor in groundtruth path.
lkt_dist = abs(scale_factor/camera_f*pos(:, 1:2) - groundtruth);
lkt_dist_th = pos(:, 4) - groundtruth_angle;

% Get the LKT and tracker results.
load([pipe_name '_comb_results.mat']);

% Find nearest neighbor in groundtruth path.
comb_dist = abs(scale_factor/camera_f*pos(:, 1:2) - groundtruth);
comb_dist_th = pos(:, 4) + groundtruth_angle;

% Plot the xy results.
% Setup the figure.
figure;
hold on;

% Plot the main results.
plot(lkt_dist(:, 1), 'b', 'LineWidth', 2);
plot(lkt_dist(:, 2), 'b--', 'LineWidth', 2);
plot(comb_dist(:, 1), 'g', 'LineWidth', 2);
plot(comb_dist(:, 2), 'g--', 'LineWidth', 2);

% Do some axis stuff.
limits = axis;

% Plot the marker position if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
    marker_dist = sqrt(sum((marker_pos(2:end, 1:2) - groundtruth).^2, 2));
    plot(marker_dist, 'r', 'LineWidth', 2);
end

% Add title, axis labels, and stuff.
title('Evaluation of LKT and Tracking Results');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('LKT', 'LKT with Tracking', 'AR Tag Estimate');

% Set the axis limits.
axis(limits + [0 0 0 1]); 

% Plot the angle results.
% Setup the figure.
figure;
hold on;

% Plot the main results.
plot(lkt_dist_th, 'b', 'LineWidth', 2);
plot(-comb_dist_th, 'g', 'LineWidth', 2);

