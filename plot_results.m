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

% Get the appropriate ground truth measures.
if strncmp(pipe_name, 'crawlerTop', 10)
    load([pipe_name '.mat']);

    groundtruth = pose(:, [3 1]);
    groundtruth_angle = pose(:, 6);
else
    load([pipe_name '_groundtruth.mat']);

    groundtruth = camera_pos(1:511, [1, 2]);
    groundtruth_angle = -camera_pos(1:511, 3);
end

% Get the LKT results.
load([pipe_name '_lkt_results.mat']);

% Find nearest neighbor in groundtruth path.
lkt_dist = abs(scale_factor/camera_f*pos(1:511, 1:2) - groundtruth);
lkt_dist_th = abs(pos(1:511, 4) - groundtruth_angle);

meow1 = sum(lkt_dist.^2, 2);
for i=1:511
    lkt_rms(i) = sqrt(sum(meow1(1:i)));
    lkt_rms_th(i) = sqrt(sum(lkt_dist_th(1:i).^2));
end

% Get the LKT and tracker results.
load([pipe_name '_comb_results.mat']);

% Find nearest neighbor in groundtruth path.
comb_dist = abs(scale_factor/camera_f*pos(1:511, 1:2) - groundtruth);
comb_dist_th = abs(pos(1:511, 4) + groundtruth_angle);

meow1 = sum(comb_dist.^2, 2);
for i=1:511
    comb_rms(i) = sqrt(sum(meow1(1:i)));
    comb_rms_th(i) = sqrt(sum(comb_dist_th(1:i).^2));
end


% Plot the xy results.
% Setup the figure.
figure;
subplot(2, 1, 1);
hold on;

% % Plot the main results.
% plot(lkt_dist(:, 1), 'b', 'LineWidth', 2);
% plot(comb_dist(:, 1), 'g', 'LineWidth', 2);
% plot(lkt_dist(:, 2), 'b-.', 'LineWidth', 2);
% plot(comb_dist(:, 2), 'g-.', 'LineWidth', 2);
% figure; hold on;
% plot(pi/180*comb_dist_th, 'g--', 'LineWidth', 2);
% plot(pi/180*lkt_dist_th, 'b--', 'LineWidth', 2);

% plot(sqrt(sum(lkt_dist.^2, 2)), 'b', 'LineWidth', 2);
% plot(sqrt(sum(comb_dist.^2, 2)), 'g', 'LineWidth', 2);

plot(lkt_rms, 'b', 'LineWidth', 2);
plot(comb_rms, 'g', 'LineWidth', 2);

% Plot the marker position if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
%     marker_dist = abs(marker_pos(2:end, 1:2) - groundtruth);
%     plot(marker_dist(:, 1), 'r', 'LineWidth', 2);
%     plot(marker_dist(:, 2), 'r-.', 'LineWidth', 2);
%     plot(sqrt(sum(marker_dist.^2, 2)), 'r', 'LineWidth', 2);
end

% Add title, axis labels, and stuff.
title('Root Mean Square Error in Position Estimation');
xlabel('Time (Frames)');
ylabel('RMS Error');
legend('LKT', 'LKT with Tracking', 'AR Tag Estimate');
xlim([0 550]);

%% Plot the angle results.
% Setup the figure.
% figure;
subplot(2, 1, 2);
hold on;

% Plot the main results.
% plot(lkt_dist_th, 'b', 'LineWidth', 2);
% plot(comb_dist_th, 'g', 'LineWidth', 2);

plot(lkt_rms_th, 'b', 'LineWidth', 2);
plot(comb_rms_th, 'g', 'LineWidth', 2);


% Plot the marker orientation if this is real data.
if ~strncmp(pipe_name, 'crawlerTop', 10)
%     marker_dist_th = abs(data.marker_orient(2:end, 4) - groundtruth_angle);
%     plot(marker_dist_th, 'r', 'LineWidth', 2);
end

% Add title, axis labels, and stuff.
title('Root Mean Square Error in Orientation Estimation');
xlabel('Time (Frames)');
ylabel('RMS Error');
legend('LKT', 'LKT with Tracking', 'AR Tag Estimate');
xlim([0 550]);

results = [sum(sum(lkt_dist.^2, 2)) std(sum(lkt_dist.^2, 2)) mean(lkt_dist_th) std(lkt_dist_th);
           sum(sum(comb_dist.^2, 2)) std(sum(comb_dist.^2, 2)) mean(comb_dist_th) std(comb_dist_th)];
