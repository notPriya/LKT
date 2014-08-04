%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

% Determine which video to use.
if ~exist('pipe_name', 'var')	
	pipe_name = input('Which crawler video should we use?   ', 's');
end

% Load the frames to process.
if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

% Initialize frame variables.
start = 1;
n = size(frames, 4) - start;

% Plotting stuff.
evaluation = true;

% Initialize the first line to track.
line_data.state = zeros(6, 1);
line_data.sigma = eye(6);
line_data.real = false;
line_data.skip = Inf;

% Initialize the weights.
weights = [0; 3; 1];
num_skips = 3;

% Scale factor to go from pixels to real world units.
scale_factor = -0.1278;
camera_f = 510;

% Initialize the position of the line.
initial_pos.xy = [];
initial_pos.index = [];
initial_pos.needs_update = true;

% Estimated position of the robot.
pos = zeros(n, 3);

% Run the edge tracker.
for i=start:start+n
    index = i-start + 1;
    
    if mod(index, 50) == 0
        disp(index);
        if ~evaluation
            close;
        end
    end
    
    % Get the preprocessed image.
    I = preprocessImage(frames(:,:,:,i), true, false, [20 10]);
    I = rot90(I, -1);
    [line_data] = edgeTracker(I, weights, num_skips, line_data, evaluation);
    
    % Update the initial position of the line if we are tracking a new
    % line.
    if line_data.real && initial_pos.needs_update
        initial_pos.xy = line_data.state(1:2);
        initial_pos.index = max(1, index-1);
        initial_pos.needs_update = false;
        line_data.skip = 0;
    end
    
    % If we have lost the line and skipped one frame, make sure we update
    % the initial pos the next time we find a line.
    if line_data.skip > num_skips
        initial_pos.needs_update = true;
    end

    % Determine the change in position. Orientation is just the orientation
    % of the line.
    delta_pos = initial_pos.xy - line_data.state(1:2);
    theta = pos(initial_pos.index, 3);
    phi = atan2d(delta_pos(1), delta_pos(2)) + 90;
    r = norm(delta_pos, 2);

    pos(index, :) = [pos(initial_pos.index, 1) - r * cosd(phi - theta) ...
                     pos(initial_pos.index, 2) + r * sind(phi - theta) ...
                     sign(line_data.state(3))*90 - line_data.state(3)];

end

%% Visualize the results
figure;
hold on;
% plot(start:start+n, scale_factor*1/camera_f*pos(:, 1:2), '--');
plot(scale_factor*1/camera_f*pos(:, 1), scale_factor*1/camera_f*pos(:, 2), 'b', 'LineWidth', 2);
plot(camera_pos(:, 1), camera_pos(:, 2));
plot(camera_pos(:, 1), scale_factor*1/camera_f*pos(2:end, 2), 'g', 'LineWidth', 2);

% plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
% plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);

%%
figure;
plot(start:start+n, -pos(:, 3), '--', 'LineWidth', 2);
hold on;
% plot(start:start+n, pose(:, 5) - pose(1, 5), 'k', 'LineWidth', 2);