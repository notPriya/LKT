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

% Get password from the user,
if ~exist('setupEmail', 'var') || ~setupEmail
    % Setup the SMTP stuff.
    password = input('Enter Password:  ', 's');
    setpref('Internet', 'E_mail', 'pdeo@andrew.cmu.edu')
    setpref('Internet', 'SMTP_Server', 'smtp.andrew.cmu.edu');
    setpref('Internet', 'SMTP_Username', 'pdeo');
    setpref('Internet', 'SMTP_Password', password);
    % Dont save the password!
    clear password;
    % Clear screen to get rid of the password.
    clc;
    % Pause matlab so user can clear password.
    input('Please clear password from command history.');
    
    % Setup the SSL connection.
    props = java.lang.System.getProperties;
    props.setProperty('mail.smtp.auth','true');
    props.setProperty('mail.smtp.socketFactory.class', ...
                      'javax.net.ssl.SSLSocketFactory');
    props.setProperty('mail.smtp.socketFactory.port','465');
    
    % Make sure we dont keep doing this.
    setupEmail = true;
end

start = 1;
n = size(frames, 4) - start;
visualize = false;
evaluation = true;

% Scale factor for the crawler.
scale_factor =  -.026720;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the LKT variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The initial assumption is that we havent transformed.
M = eye(3,3);
templateData = [];
distance_threshold = .1;

% Set the kind of warp we are using.
warp = getRigidBodyWarp();

% Get the odometry bounding box.
if ~exist('odom_rect', 'var')
    imshow(frames(:,:,:,5));
    [x, y] = ginput(2);
    odom_rect = [min(size(frames, 2)-100, max(1, x'-50)) ...
                 min(size(frames, 1)-100, max(1, y'-50))];
    odom_rect = int16(odom_rect);
    clear x y;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the Joint         %
% tracking variables           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants based on pipe video.
camera_f = 510;  % MAGIC
% Scale factor to go from pixels to real world units.
pixel_scale_factor = 6;  % MAGIC

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
TrackedObject.lkt_pos = zeros(n, 3);
TrackedObject.jt_pos = zeros(n, 3);
TrackedObject.pos = zeros(n, 3);
% Time results.
TrackedObject.time = zeros(n, 1);
% LKT Results
TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 3);
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
        LucasKanade(frames(50:end-50, 50:end-50, :, i), ...
                    frames(50:end-50, 50:end-50, :, i+1), ...
                    M, warp, templateData, odom_rect);
    
    % If we got some bad data, get rid of it.
    if abs((1 - M(1,1))*510*(1/250)) > 1.5 * distance_threshold
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
    % Distance Estimation
    %%%%%%%%%%%%%%%%%%%%%
    % Get the scaling factor to estimate position from LKT
    alpha = M(1, 1);
    lkt_pos = [M(1, 3)/alpha; M(2, 3)/alpha; (1 - alpha)*510*(1/250)] + ...
               TrackedObject.template_pos(index, :)';
    
    % Update the initial position of the line if we are tracking a new
    % line.
    if line_data.real && initial_pos.needs_update
        initial_pos.xy = line_data.state(1:2);
        initial_pos.pos = TrackedObject.pos(max(1, index-1), 1:2);
        initial_pos.needs_update = false;
    end
    
    % Determine the change in position. Orientation is just the orientation
    % of the line.
    delta_pos = initial_pos.xy - line_data.state(1:2);
    jt_pos = [initial_pos.pos'+delta_pos; lkt_pos(3)];
    
    % Use the joint tracking if the circle is real and the position
    % doesnt need to be updated.
    gamma = 0.5; % Mixing factor.
    
    % Combine the LKT position estimate and the joint tracking position
    % estimate.
    pos = gamma * lkt_pos + (1-gamma) * jt_pos;
    
    % Dont save the line tracking position if its wrong. We make it a NaN
    % so the vector plots as usual but nothing shows up. Do this last to
    % keep everything from going NaN. 
    if ~line_data.real
        jt_pos = NaN;
    end
    
    %%%%%%%%%%%%%%%%%%%%%
    % Save Results.
    %%%%%%%%%%%%%%%%%%%%%
    % Add everything to the result
    % Distance results
    TrackedObject.lkt_pos(index, :) = lkt_pos;
    TrackedObject.jt_pos(index, :) = jt_pos;
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
        alpha = 1 - 250 * 1/510 * (pos(3) - TrackedObject.template_pos(index, 3));
        M(1,1) = alpha;
        M(2,2) = alpha;
    %     psi = 0.5;
    %     M(1:2, 3) = psi * M(1:2, 3) + (1-psi) * pos(1:2);
    end
        
    % If we have moved past a threshold re-initialize the template.
    if abs(TrackedObject.pos(index, 3)-TrackedObject.template_pos(index, 3)) > distance_threshold
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
save([pipe_name '_comb_results.mat'], 'pos');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if visualize
    for i = start:start+n-1
        % Extract the image.
        I = preprocessImage(frames(50:end-50,50:end-50,:,i+1), true, false);
        I_full = preprocessImage(frames(:,:,:,i+1), true, false);

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
        imshow(uint8(I_full));
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
figure;
hold on;
plot(start:start+n-1, scale_factor*TrackedObject.pos, '--', 'LineWidth', 2);
plot(start:start+n-1, scale_factor*TrackedObject.jt_pos, '-.');
plot(start:start+n-1, scale_factor*TrackedObject.lkt_pos, '*');

plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);
plot(start:start+n, pose(:, 2) - pose(1, 2), 'r', 'LineWidth', 2);

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
                  'Testing the pipeline for crawler with tracking.' ...
                 ];

message = sprintf(message_format, pipe_name, start, start+n-1, mean(TrackedObject.time), sum(TrackedObject.time));


% emailResults(message, figure_filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean up environment.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear I M T alpha distance_threshold error i template templateData index ...
			ground_truth gamma circle camera_f small_delta_radius_guess small_radius_guess ...
			weights update_pos ration pipe_radius initial_pos jt_pos lkt_pos delta pos ...
			ratio date_string figure_filename message_format message;
