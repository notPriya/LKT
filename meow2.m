pipe_name = 'trial9';
n=1281;

% Create the first figure.
figure;
hold on;

if strncmp(pipe_name, 'trial8', 6)
    ground_truth = [[1:n]' linspace(0, 0.4445, n)'];
else
    ground_truth = [ [1:461]' linspace(0, 0.17145, 461)'; [462:700]' linspace(0.17145, 0.17145, 239)'; [701:n]' linspace(0.17145, 0.3175, 581)'];
end

% Plot the LKT Results.
load([pipe_name '_lkt_result.mat']);
lkt_error = abs( pos(:, 3) - ground_truth(2:end-1, 2));

for i=1:n-2
    lkt_rms(i) = sqrt(sum(lkt_error(1:i).^2));
end

% Plot the LKT + Joint Tracking results.
load([pipe_name '_comb_results.mat']);
comb_error = abs(pos(:, 3) - ground_truth(2:end-1, 2));

for i=1:n-2
    comb_rms(i) = sqrt(sum(comb_error(1:i).^2));
end

if ~exist('pos2', 'var')
    % load(['../matlabSnakeControl/vc_' pipe_name '_data2.mat']);
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
mm_error = abs(pos2 - ground_truth(:, 2));

for i=1:n
    mm_rms(i) = sqrt(sum(mm_error(1:i).^2));
end


% % Plot all the results.
% plot(lkt_error, 'b', 'LineWidth', 2);
% plot(comb_error, 'g', 'LineWidth', 2);
% plot(mm_error, 'r', 'LineWidth', 2);

% Plot all the results.
plot(lkt_rms, 'b', 'LineWidth', 2);
plot(comb_rms, 'g', 'LineWidth', 2);
plot(mm_rms, 'r', 'LineWidth', 2);

% Put the title, axis labels, etc.
% title('Error over time for the Snake Robot in Pipes');
title('Root Mean Squared Error for the Snake Robot in Pipes');
xlabel('Time (Frames)');
% ylabel('Error (m)');
ylabel('RMS Error');
legend('LKT', 'LKT with Joint Tracking', 'Motion Model', 'Expected Motion')