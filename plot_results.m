pipe_name = 'trial8';
n=1281;

% Create the first figure.
figure;
hold on;

if strcmp(pipe_name, 'trial8')
    ground_truth = [0 0; n 0.4445];
else
    ground_truth = [0 0; 245 0.17145; 450 0.17145; n 0.3175];
end

% Plot the LKT Results.
load([pipe_name '_lkt_result.mat']);
plot(pos(:, 3)*0.3048*0.22, 'b', 'LineWidth', 2);

% Plot the LKT + Joint Tracking results.
load([pipe_name '_comb_results.mat']);
plot(pos(:, 3)*0.3048*0.22, 'g', 'LineWidth', 2);

if ~exist('pos2', 'var')
%     load(['../matlabSnakeControl/vc_' pipe_name '_data2.mat']);
    load(['vc_' pipe_name '_data2.mat']);

    % Get the State Estimation Motion data.
    T = eye(4);
    pos2 = [];
    for i=1:n
    T = T * recordLog.misc.Motions{i};
    pos2 = [pos2; T(1, 4)];
    end
end

% Plot the State Estimation results.
plot(pos2, 'r', 'LineWidth', 2);

% Plot the estimated groundtruth.
plot(ground_truth(:, 1), ground_truth(:, 2), 'k--', 'LineWidth', 2);

% Put the title, axis labels, etc.
title('Visual Odometry for the Snake Robot in Pipes');
xlabel('Frame Number');
ylabel('Distance Traveled (m)');
legend('LKT', 'LKT with Joint Tracking', 'State Estimation', 'Expected Motion')

