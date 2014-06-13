if ~exist('frames', 'var')
    load('crawlerTopXY.mat');
end

% Initialize frame variables.
start = 1;
n = size(frames, 4) - start;

% Plotting stuff.
evaluation = false;

% Initialize the first line to track.
line_data.state = zeros(6, 1);
line_data.sigma = eye(6);
line_data.real = false;

% Initialize the weights.
weights = [0; 3; 1];

% Scale factor to go from pixels to real world units.
scale_factor = 6;
camera_f = 510;

% Initialize the position of the line.
initial_pos.xy = [];
initial_pos.index = [];
initial_pos.needs_update = true;

% Estimated position of the robot.
pos = zeros(n, 3);

% Run the edge tracker.
for i=start:start+n-1
    index = i-start + 1;
    
    % Get the preprocessed image.
    I = preprocessImage(frames(:,:,:,i), true, false);
    [line_data] = edgeTracker(I, weights, line_data, evaluation);
    
    % Update the initial position of the line if we are tracking a new
    % line.
    if line_data.real && initial_pos.needs_update
        initial_pos.xy = line_data.state(1:2);
        initial_pos.index = index;
        initial_pos.needs_update = false;
    end
    
    % If we have lost the line, make sure we update the initial pos the
    % next time we find a line.
    if ~line_data.real
        initial_pos.needs_update = true;
    end

    % Determine the change in position. Orientation is just the orientation
    % of the line.
    delta_pos = 1/camera_f * (line_data.state(1:2) - initial_pos.xy);
    pos(index+1, :) = [pos(initial_pos.index, 1:2)+delta_pos' line_data.state(3)+90];
end

%% Visualize the results
figure;
plot(start:start+n, scale_factor*pos(:, 1:2));
hold on;
plot(start:start+n, pose(:, 3) - pose(1, 3), 'b--');
plot(start:start+n, pose(:, 1) - pose(1, 1), 'g--');
plot(start:start+n, pose(:, 2) - pose(1, 2), 'r--');
