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
% Get accel and gyro values    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the accelerometer and gyroscope values using the groundtruth
% position information.
accel = diff(pose(:, [3 1 2]), 2);
accel = interp1(accel, -1:n-1, 'cubic', 'extrap');
accel(2:3, 3) = 0;  % Weird thing about the first position reading.
gyro = diff(pose(:, 5));
gyro = interp1(gyro, 0:n, 'cubic', 'extrap')';

% Add noise to the measurements.
accel = accel + normrnd(0, 0.01, size(accel));
gyro = gyro + normrnd(0, 0.01, size(gyro));

% Get the unfiltered IMU position estimates for comparison.
true_imu_pos = [0 0 0 0];
vel = [0 0 0];
for i=1:n+1
    vel = vel + accel(i, :);
    true_imu_pos(i+1, :) = true_imu_pos(i, :) + [vel gyro(i)];
end
true_imu_pos = true_imu_pos(2:end, :);
clear vel i;

% Show the IMU position estimate.
figure;
hold on;
plot(start:start+n, true_imu_pos(:, 1:3), '*--', 'LineWidth', 2);
plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);
plot(start:start+n, pose(:, 2) - pose(2, 2), 'r', 'LineWidth', 2);
drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the IMU variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
velocity = [0 0 0];

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

% Initialize the first line to track.
line_data.state = zeros(6, 1);
line_data.sigma = eye(6);
line_data.real = false;

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
TrackedObject.imu_pos = zeros(n, 4);
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
% IMU Results
TrackedObject.velocity = zeros(n, 3);


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
    line_data = edgeTracker(I, weights, line_data, evaluation);
    
    t = toc;
    fprintf('Elapsed time is %f seconds.\n', t);
    if mod(i, 200) == 0
        disp(i);
    end

    %%%%%%%%%%%%%%%%%%%%%
    % IMU Estimation
    %%%%%%%%%%%%%%%%%%%%%
    % Integrate to get the robot velocity.
    velocity = velocity + accel(index, :);
    
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
          
    % Use the IMU to estimate position.
    imu_pos = TrackedObject.pos(max(1, index-1), :)' + camera_f/scale_factor * [velocity'; gyro(index)];
    
    % Use the joint tracking if the circle is real and the position
    % doesnt need to be updated.
    alpha = [0.5; 0.4; 0.5; 0.1]; % Mixing factor for LKT.
    beta = [0.3; 0.3; 0.5; 0.3]; % Mixing factor for gyros.
    
    if any(alpha+beta > 1)
        error('Error in setting alpha and beta.');
    end
    
    % Combine the LKT position estimate and the joint tracking position
    % estimate.
    pos = alpha .* lkt_pos + (1-alpha-beta) .* jt_pos + beta .* imu_pos;
        
    %%%%%%%%%%%%%%%%%%%%%
    % Save Results.
    %%%%%%%%%%%%%%%%%%%%%
    % Add everything to the result
    % Distance results
    TrackedObject.lkt_pos(index, :) = lkt_pos;
    TrackedObject.jt_pos(index, :) = jt_pos;
    TrackedObject.imu_pos(index, :) = imu_pos;
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
    % IMU Results
    TrackedObject.velocity(index, :) = velocity;
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the template.
    %%%%%%%%%%%%%%%%%%%%%
    % Update the information from the position, if the line is real.
    if line_data.real
        alpha = 1 - 1/camera_f * (pos(3) - TrackedObject.template_pos(index, 3));
        M(1,1) = alpha;
        M(2,2) = alpha;
    %     psi = 0.5;
    %     M(1:2, 3) = psi * M(1:2, 3) + (1-psi) * pos(1:2);
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
    % If the lines are real, incorporate the LKT information into the
    % state.
    phi = 1;
    if line_data.real
        phi = 0.9;
    end
%     line_data.state(1:2) = phi*line_data.state(1:2) + ...
%         (1-phi)*(camera_f*1/pixel_scale_factor*(pos(1:2) - initial_pos.pos')+initial_pos.xy);
        
    % If we have lost the line, make sure we update the initial pos the
    % next time we find a line.
    if ~line_data.real
        initial_pos.needs_update = true;
    end
end

%% Save off the tracked information
pos = TrackedObject.pos;
save([pipe_name '_imu_results.mat'], 'pos', 'accel', 'gyro');

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
plot(start:start+n-1, scale_factor*1./camera_f*pos, '--');

plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);
plot(start:start+n, pose(:, 2) - pose(2, 2), 'r', 'LineWidth', 2);

%%
figure;
hold on;
plot(start:start+n-1, -TrackedObject.pos(:, 4), '--', 'LineWidth', 2);

plot(start:start+n, pose(:, 5) - pose(1, 5), 'b', 'LineWidth', 2);

%% Save to a PNG file with today's date and time.
date_string = datestr(now,'yy_mm_dd_HH_MM');
figure_filename = [pipe_name '_results_' date_string '.png'];
saveas(gcf, figure_filename , 'png');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the z-axis motion %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

message_format = ['Video:\t\t\t%s\n' ...
                  'Start Frame:\t\t%d\n' ...
                  'End Frame:\t\t%d\n\n' ...
                  'Average Time:\t\t%f\n\n' ...
                  'Total Time:\t\t%f\n\n' ...
                  'ExtraInfo:\n\n' ...
                  'Testing the pipeline for crawler with tracking.' ...
                 ];

message = sprintf(message_format, pipe_name, start, start+n-1, mean(TrackedObject.time), sum(TrackedObject.time));

emailResults(message, figure_filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean up environment.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear I M T alpha distance_threshold error i template templateData index ...
			ground_truth gamma circle small_delta_radius_guess small_radius_guess ...
			weights update_pos ration pipe_radius initial_pos jt_pos lkt_pos delta pos ...
			ratio date_string figure_filename message_format message;
