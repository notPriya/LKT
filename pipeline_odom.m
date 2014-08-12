%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

addpath ./toolbox
addpath ./line_tracker

% Determine which video to use.
if ~exist('pipe_name', 'var')	
	pipe_name = input('Which crawler video should we use?   ', 's');
end

% Load the frames to process.
if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

% Setup the email client.
setup_mail;

% Initialize frame variables.
start = 1;
n = size(frames, 4) - start;

% Plotting stuff.
visualize = false;
evaluation = true;

% Scale factor for the crawler. Goes between vision units and real world
% units.
scale_factor =  -5;  % MAGIC
% Constants based on pipe video.
camera_f = 510;  % MAGIC

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the odometry      %
% variables.                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate some stats on the odometry.
delta = pose(2:end, [3 1 2]) - pose(1:end-1, [3 1 2]);
sigma = min(0.5, max(0.01, std(delta)));

% Add gaussian noise to the delta.
delta = delta + [normrnd(0, 2*sigma(1), size(delta, 1), 1) ...
                 normrnd(0, 2*sigma(2), size(delta, 1), 1) ...
                 normrnd(0, 2*sigma(3), size(delta, 1), 1)];

% Integrate to get noisy odometry.
odometry = [0 0 0];
for i=2:length(delta)
    odometry(i, :) = odometry(i-1, :) + delta(i-1, :);
end

% Put into scale.
odometry = odometry .* camera_f/scale_factor;

clear delta sigma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the IMU variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate some stats on the orientation.
delta = pose(2:end, 5) - pose(1:end-1, 5);
sigma = min(0.5, max(0.01, std(delta)));

% Add gaussian noise to the delta.
delta = delta + normrnd(0, sigma, size(delta));

% Integrate to get noisy pose estimates.
imu_pose = 0;
for i=2:length(delta)
    imu_pose(i, :) = imu_pose(i-1, :) + delta(i-1, :);
end

% Put into correct scale.
imu_pose = -imu_pose;

clear delta sigma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the LKT variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The initial assumption is that we havent transformed.
M = eye(3,3);
templateData = [];
distance_threshold = 10;

% Set the kind of warp we are using.
warp = getRigidBodyWarp();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the Joint         %
% tracking variables           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Weights on the features for picking best measurement.
weights = [0; 3; 1];
num_skips = 1;

% Initialize the first line to track.
line_data.state = zeros(6, 1);
line_data.sigma = eye(6);
line_data.real = false;
line_data.skip = Inf;

% Initialize the position of the line.
initial_pos.xy = [];
initial_pos.pos = [];
initial_pos.needs_update = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the result struct %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a structure for saving the results.
TrackedObject.lkt_pos = zeros(n, 4);
TrackedObject.jt_pos = zeros(n, 4);
TrackedObject.sensor_pos = zeros(n, 4);
TrackedObject.pos = zeros(n, 4);
% Time results.
TrackedObject.time = zeros(n, 1);
% LKT Results
TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 4);
% Joint Tracking Results.
TrackedObject.lines = cell(n, 1);
TrackedObject.initial_pos = cell(n, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For each image run the LKT  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = start:start+n-1
    index = i-start + 1;

    %%%%%%%%%%%%%%%%%%%%%
    % Lucas Kanade
    %%%%%%%%%%%%%%%%%%%%%
    tic;
    % Run Lucas-Kanade Tracker 
    [M, templateData, error] = ...
        LucasKanade(frames(:, :, :, i), ...
                    frames(:, :, :, i+1), ...
                    M, warp, templateData);
    
    % If we got some bad data, get rid of it.
    if abs((1 - M(1,1))*camera_f) > 3 * distance_threshold
        M = TrackedObject.M(:, :, index-1);  % Use the old M.
    end
            
    %%%%%%%%%%%%%%%%%%%%%
    % Line Tracking
    %%%%%%%%%%%%%%%%%%%%%
    % Get the preprocessed image.
    I = preprocessImage(frames(:,:,:,i), true, false);
    line_data = edgeTracker(I, weights, num_skips, line_data, evaluation);
    
    t = toc;
    fprintf('Elapsed time is %f seconds.\n', t);
    if mod(i, 200) == 0
        disp(i);
    end
    
    %%%%%%%%%%%%%%%%%%%%%
    % Distance Estimation
    %%%%%%%%%%%%%%%%%%%%%
    % Get the scaling factor to estimate position from LKT
    alpha = M(1, 1);
    lkt_pos = [M(1, 3)/alpha; M(2, 3)/alpha; (alpha - 1)*camera_f; M(1, 2)/alpha*(180/pi)] + ...
               TrackedObject.template_pos(index, :)';
    
    % Update the initial position of the line if we are tracking a new
    % line.
    if line_data.real && initial_pos.needs_update
        initial_pos.xy = line_data.state(1:2);
        initial_pos.pos = TrackedObject.jt_pos(max(1, index-1), :);
        initial_pos.needs_update = false;
        line_data.skip = 0;
    end
    
    % Determine the change in position. Orientation is just the orientation
    % of the line.
    delta_pos = initial_pos.xy - line_data.state(1:2);
    theta = initial_pos.pos(4);
    phi = atan2d(delta_pos(1), delta_pos(2)) + 90;
    r = norm(delta_pos, 2);
    jt_pos = [initial_pos.pos(1) - r * cosd(phi - theta); ...
              initial_pos.pos(2) + r * sind(phi - theta); ...
              0; ...
              sign(line_data.state(3))*90 - line_data.state(3)];
          
    % Distance estimation using the encoder and IMU sensors.
    sensor_pos = [odometry(index, :)'; imu_pose(index)];
    
    % Use the joint tracking if the circle is real and the position
    % doesnt need to be updated.
    alpha = [0.6; 0.5; 0.6; 0]; % LKT mixing factor.
    beta = [0.3; 0.3; 0.4; 0.3]; % Sensor mixing factor.
    
    % If we messed up setting the weights throw an error.
    if any(alpha+beta > 1)
        error('Error in setting mixing weights alpha and beta.');
    end
    
    % Combine the LKT position estimate and the joint tracking position
    % estimate.
    pos = alpha .* lkt_pos + (1-alpha-beta) .* jt_pos + beta .* sensor_pos;
        
    %%%%%%%%%%%%%%%%%%%%%
    % Save Results.
    %%%%%%%%%%%%%%%%%%%%%
    % Add everything to the result
    % Distance results
    TrackedObject.lkt_pos(index, :) = lkt_pos;
    TrackedObject.jt_pos(index, :) = jt_pos;
    TrackedObject.sensor_pos(index, :) = sensor_pos;
    TrackedObject.pos(index, :) = pos;
    % Time results.
    TrackedObject.time(index) = t;
    % LKT Results
    TrackedObject.M(:, :, index) = M;
    TrackedObject.error{index} = error;
    TrackedObject.template{index} = templateData;
    TrackedObject.template_pos(index+1, :) = TrackedObject.template_pos(index, :);
    % Line tracking results
    TrackedObject.lines{index} = line_data;
    TrackedObject.initial_pos{index} = initial_pos;
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the template.
    %%%%%%%%%%%%%%%%%%%%%
    % Update the information from the position, if the line is real.
    if line_data.real
        alpha = 1 - 1/camera_f * (pos(3) - TrackedObject.template_pos(index, 3));
        M(1,1) = alpha;
        M(2,2) = alpha;
    end
        
    % If we have moved past a threshold re-initialize the template.
    if any(abs(TrackedObject.pos(index, :)-TrackedObject.template_pos(index, :)) > distance_threshold)
        % Clear the template and M.
        templateData = [];
        M = eye(3);
        % Update the template's position.
        TrackedObject.template_pos(index+1, :) = pos;
    end
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the lines.
    %%%%%%%%%%%%%%%%%%%%%
    % If we have lost the line, make sure we update the initial pos the
    % next time we find a line.
    if line_data.skip > num_skips
        initial_pos.needs_update = true;
    end
end

%% Save off the tracked information
pos = TrackedObject.pos;
save([pipe_name '_odom_results.mat'], 'pos', 'odometry', 'theta');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if visualize
    for i = start:start+n-1
        % Extract the image.
        I = preprocessImage(frames(:,:,:,i+1), true, false);

        % Determine the optimal affine transform.
        M = TrackedObject.M(:, :, i-start+1);
        T = TrackedObject.template{i-start+1}.template;
        T = uint8(reshape(T, size(I, 1), size(I, 2)));

        % Warp the image to fit the template.
        template = warp.doWarp(I, M);

        % Smash it back into an image.
        template = uint8(template);

        % Show the template.
        subplot(2, 2, 1);
        imshow(template);
        title(i);
        
        % Draw the lines.
        subplot(2, 2, 2);
        imshow(uint8(I));
        hold on;
        line = TrackedObject.lines{i-start+1};
        vislines(line);

        % Show deviations from the original template.
        subplot(2, 2, 3);
        imagesc(abs(T - template));

        pause(0.3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the robot motion  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load([pipe_name '_comb_results.mat']);

figure;
hold on;
plot(start:start+n-1, scale_factor*1./camera_f*TrackedObject.pos(:, 1:3), '--', 'LineWidth', 2);
plot(start:start+n-1, scale_factor*1./camera_f*pos(:, 1:3), '.-');
plot(start:start+n-1, scale_factor*1./camera_f*odometry, '--');

plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);
plot(start:start+n, pose(:, 2) - pose(1, 2), 'r', 'LineWidth', 2);

%%
figure;
hold on;
plot(start:start+n-1, -TrackedObject.pos(:, 4), '--', 'LineWidth', 2);
plot(start:start+n-1, -pos(:, 4), 'r.-');
plot(start:start+n-1, -imu_pose, 'm--');

plot(start:start+n, pose(:, 5) - pose(1, 5), 'b', 'LineWidth', 2);


%% Save to a PNG file with today's date and time.
date_string = datestr(now,'yy_mm_dd_HH_MM');
figure_filename = [pipe_name '_results_' date_string '.png'];
% saveas(gcf, figure_filename , 'png');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the z-axis motion %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

message_format = ['Video:\t\t\t%s\n' ...
                  'Start Frame:\t\t%d\n' ...
                  'End Frame:\t\t%d\n\n' ...
                  'Average Time:\t\t%f\n\n' ...
                  'Total Time:\t\t%f\n\n' ...
                  'ExtraInfo:\n\n' ...
                  'Adding odometry and imu information to the pipeline for ' ...
                  'sensor-based position estimation when it is available.' ...
                 ];

message = sprintf(message_format, pipe_name, start, start+n-1, mean(TrackedObject.time), sum(TrackedObject.time));


% emailResults(message, figure_filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean up environment.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear I M alpha beta date_string delta_pos distance_threshold error ...
    evaluation figure_filename i index initial_pos jt_pos lkt_pos line_data ...
    message message_format phi r sensor_pos t templateData theta visualize ...
    weights;