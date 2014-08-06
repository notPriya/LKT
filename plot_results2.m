% Which results to plot.
pipe_name = 'pattern_1';

% Get the correct scale factor.
if strncmp(pipe_name, 'crawlerTop', 10)
    scale_factor = -5;
elseif strcmp(pipe_name, 'pattern_1')
    scale_factor = -0.1546;
elseif strcmp(pipe_name, 'pattern_2')
    scale_factor = -0.1590;
else 
    scale_factor = -0.1278;
end
% Get the camera focal length.
camera_f = 510;

% Plot the xy results.
% Setup the figure.
figure;
hold on;

% Get the appropriate ground truth measures.
if strncmp(pipe_name, 'crawlerTop', 10)
    load([pipe_name '.mat']);

    plot(pose(:, 3), pose(:, 1), 'k', 'LineWidth', 2);
else
    load([pipe_name '_groundtruth.mat']);

    plot(camera_pos(:, 1), camera_pos(:, 2), 'k--', 'LineWidth', 2);
end

% Get the LKT results.
load([pipe_name '_lkt_results.mat']);
plot(scale_factor/camera_f*pos(:, 1), scale_factor/camera_f*pos(:, 2), 'b', 'LineWidth', 2);

% Get the LKT and tracker results.
load([pipe_name '_comb_results.mat']);
plot(scale_factor/camera_f*pos(:, 1), scale_factor/camera_f*pos(:, 2), 'g', 'LineWidth', 2);


% Plot the marker position if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
    if ~exist('data', 'var')
        load([pipe_name '.mat']);
    end
    
    plot(data.marker_pos(:, 1)-data.marker_pos(1, 1), -data.marker_pos(:, 2)+data.marker_pos(1, 2), 'r', 'LineWidth', 2);
end

% Add title, axis labels, and stuff.
title('Position Estimate of LKT and Tracking');
xlabel('X Distance Traveled (m)');
ylabel('Y Distance Traveled (m)');
legend('Expected Motion', 'LKT', 'LKT with Tracking', 'AR Tag Estimate');

%% Plot the angle results.
% Setup the figure.
figure;
hold on;

if size(camera_pos, 2) > 2
    plot(-camera_pos(:, 3), 'k--', 'LineWidth', 2);
end

% Get the LKT results.
load([pipe_name '_lkt_results.mat']);
plot(pos(:, 4), 'b', 'LineWidth', 2);

% Get the LKT and tracker results.
load([pipe_name '_comb_results.mat']);
plot(-pos(:, 4), 'g', 'LineWidth', 2);

% Plot the marker orientation if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
    plot(180/pi*data.marker_orient(:, 3), 'r', 'LineWidth', 2);
end

% Add title, axis labels, and stuff.
title('Orientation estimation of LKT and Tracking');
xlabel('Time (frames)');
ylabel('Estimated Orientation (degrees)');
legend('Expected Motion', 'LKT', 'LKT with Tracking', 'AR Tag Estimate');

