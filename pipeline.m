%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

addpath ./toolbox
addpath ./joint_tracker

% pipe_name = 'pipe6';

% Load the frames to process.
if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

% Setup the email client
setup_mail;

start = 1;
n = size(frames, 4) - start;
visualize = false;
evaluation = true;

% Constant for the amount of the image we ignore,
border_size = 10;

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
    disp('Outline the odometry bounding box');
    imshow(frames(:,:,:,5));
    [x, y] = ginput(2);
    odom_rect = [min(size(frames, 2)-2*border_size, max(1, x'-border_size)) ...
                 min(size(frames, 1)-2*border_size, max(1, y'-border_size))];
    odom_rect = int16(odom_rect);
    clear x y;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the Joint         %
% tracking variables           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants based on pipe video.
pipe_radius = 3;
camera_f = 510;  % MAGIC

% Weights on the features for picking best measurement.
weights = [1.73962175432486;0;4;3.34010706169493;6.71403253431558];

% Small circle intialization.
small_radius_guess = 55;  % MAGIC.
small_delta_radius_guess = .3; % MAGIC.

if ~exist('init_state', 'var')
    disp('Outline the initial circle state.');
    % Get the initialization of the first circle.
    I = frames(:, :, :, start+1);
    imshow(I);
    [x,y] = ginput(2);

    % Create the first state from the initialization.
    c = [x(1); y(1)];
    r = sqrt( (x(1)-x(2))^2 + (y(1)-y(2))^2 );
    init_state = [c; r; 0; 0; 0];
    clear c r x y;
end

% Initialize circle.
circle.state = init_state;
circle.sigma = diag([10 10 10 5 5 5]);
circle.real = true;

% Keep track of the initial position of the circle.
initial_pos = [0 0 0];
update_pos = false;

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
TrackedObject.circles = cell(n, 1);


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
        LucasKanade(frames(border_size:end-border_size, border_size:end-border_size, :, i), ...
                    frames(border_size:end-border_size, border_size:end-border_size, :, i+1), ...
                    M, warp, templateData, odom_rect);
    
    % If we got some bad data, get rid of it.
    if abs((1 - M(1,1))*510*(1/250)) > 1.5 * distance_threshold
        M = TrackedObject.M(:, :, index-1);  % Use the old M.
    end
            
    %%%%%%%%%%%%%%%%%%%%%
    % Joint Tracking
    %%%%%%%%%%%%%%%%%%%%%
    % HACK: if the circle is not real, move it towards the black blob in
    % the image.
    if ~circle.real
        ind = findDarkRegions(frames(:, :, :, i+1));
        centroid = median(ind)';  % More outlier insensitive than mean.
        velocity = (centroid - circle.state(1:2))/100;
        circle.state(4:5) = velocity;
    end
    [circle, jt_error] = pipeJointTracker(frames(:, :, :, i+1), weights, circle, evaluation);
    
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
    
    % Get the scale ratio to estimate position from joint tracking.
    ratio = pipe_radius/circle.state(3);
    
    % Convert measurements from pixels to deci-feet.
    delta = .1* [(circle.state(1) - size(frames, 2)/2) * ratio; ...
                 (circle.state(2) - size(frames, 1)/2) * ratio; ...
                  ratio * camera_f];
    
    % Initialize the position.
    if i==start
        initial_pos = delta;
    end 
    
    % Get the joint tracking position.
    jt_pos = initial_pos - delta;
    
    % Use the joint tracking if the circle is real and the position
    % doesnt need to be updated.
    gamma = 1; % Mixing factor.
    if circle.real && ~update_pos
        gamma = 0.6;
    end
    
    % Combine the LKT position estimate and the joint tracking position
    % estimate.
    pos = gamma * lkt_pos + (1-gamma) * jt_pos;
    
    % Dont save the joint tracking position if its wrong. We make it a NaN
    % so the vector plots as usual but nothing shows up. Do this last to
    % keep everything from going NaN. 
    if ~circle.real || update_pos
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
    % Joint tracking results
    TrackedObject.circles{index} = circle;
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the template.
    %%%%%%%%%%%%%%%%%%%%%
    % If the circle is real, incorporate that info into M.
    if circle.real && ~update_pos
        alpha = 1 - 250 * 1/510 * (pos(3) - TrackedObject.template_pos(index, 3));
        M(1,1) = alpha;
        M(2,2) = alpha;
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
    % Updating the circles.
    %%%%%%%%%%%%%%%%%%%%%
    % If the circle is real, incorporate the LKT information into the
    % state.
    phi = 1;
    if circle.real
        phi = 0.7;
    end
%     circle.state(1:3) = phi*circle.state(1:3)+(1-phi).*(.1*pipe_radius*camera_f./(initial_pos(1:3) - pos(1:3)));
    circle.state(3) = phi*circle.state(3)+(1-phi).*(.1*pipe_radius*camera_f./(initial_pos(3) - pos(3)));
    
    % If we are tracking too big a circle reinitialize it.
    % Or if the circle goes off the screen reinitialize it.
    if circle.state(3) > 240 || ...
       ~(circle.state(1) > 0 && circle.state(1) < size(frames, 2) && circle.state(2) > 0 && circle.state(2) < size(frames, 1))
        % Reinitialize the circle.
        circle.state(3) = small_radius_guess;
        circle.state(6) = small_delta_radius_guess;
        circle.real = false;

        % Get an estimate of the current circle's position to continue
        % updating the radius.
        % Get the scale ratio to estimate position from joint tracking.
        ratio = pipe_radius/circle.state(3);
        % Convert measurements from pixels to deci-feet.
        delta = .1* [(circle.state(1) - size(frames, 2)/2) * ratio; ...
                     (circle.state(2) - size(frames, 1)/2) * ratio; ...
                      ratio * camera_f];
        initial_pos = pos + delta;
        
        % Set the flag to update pos, once the circle is real.
        update_pos = true;
    end
    
    % Update the initial position, once the circle is real.
    if circle.real && update_pos
        initial_pos = pos + delta;
        update_pos = false;
    end
end

%% Save off the tracked information
pos = TrackedObject.pos;
save([pipe_name '_comb_results.mat'], 'pos');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if visualize
    for i = start:start+n-1
        % Extract the image.
        I = preprocessImage(frames(border_size:end-border_size, border_size:end-border_size, :,i+1), true, false);

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
        
        % Draw the circles.
        subplot(2, 2, 2);
        imshow(frames(:, :, :, i+1));
        circle = TrackedObject.circles{i-start+1};
        if circle.real
            viscircles(circle.state(1:2)', circle.state(3), 'EdgeColor', 'b');
        else
            viscircles(circle.state(1:2)', circle.state(3), 'EdgeColor', 'k');
        end


        % Show deviations from the original template.
        subplot(2, 2, 3);
        imagesc(abs(T - template));

        pause(0.3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the z-axis motion %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load([pipe_name '_groundtruth.mat']);
figure;
plot(start:start+n-1, TrackedObject.pos(:, 3)*0.3048*0.22, 'b');
% plot(start:start+n-1, TrackedObject.pos(:, 3), 'b');
hold on;
plot(start:start+n-1, TrackedObject.lkt_pos(:, 3)*0.3048*0.22, 'm');
plot(start:start+n-1, TrackedObject.jt_pos(:, 3)*0.3048*0.22, 'g');
% plot(start:start+n-1, (ground_truth(start:start+n-1) - ground_truth(start))/10, 'k')
% axis([0 start+n-1 0 max(ground_truth-ground_truth(start))/10]);

%% Save to a PNG file with today's date and time.
date_string = datestr(now,'yy_mm_dd_HH_MM');
figure_filename = [pipe_name '_results_' date_string '.png'];
saveas(gcf, figure_filename , 'png');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Send Email to the user.     %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

message_format = ['Video:\t\t\t%s\n' ...
                  'Start Frame:\t\t%d\n' ...
                  'End Frame:\t\t%d\n\n' ...
                  'Average Time:\t\t%f\n\n' ...
                  'Total Time:\t\t%f\n\n' ...
                  'ExtraInfo:\n\n' ...
                  'Testing out the videos from the actual robot.' ...
                 ];

message = sprintf(message_format, pipe_name, start, start+n-1, mean(TrackedObject.time), sum(TrackedObject.time));


emailResults(message, figure_filename);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean up environment.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear I M T alpha distance_threshold error i template templateData index ...
			ground_truth gamma circle camera_f small_delta_radius_guess small_radius_guess ...
			weights update_pos ration pipe_radius initial_pos jt_pos lkt_pos delta pos ...
			ratio date_string figure_filename message_format message;
