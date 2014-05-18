%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

addpath ./toolbox
addpath ./joint_tracker

pipe_name = 'pipe1';

if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

start = 1;
n = (size(frames, 4)-1);
visualize = false;
evaluation = true;

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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the Joint         %
% tracking variables           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constants based on pipe video.
pipe_radius = 5;
camera_f = 510;  % MAGIC

% Weights on the features for picking best measurement.
weights = [1.73962175432486;0;3;3.34010706169493;6.71403253431558];

% Small circle intialization.
small_radius_guess = 55;  % MAGIC.
small_delta_radius_guess = .3; % MAGIC.

if ~exist('init_state', 'var')
    % Get the initialization of the first circle.
    I = frames(:, :, :, start);
    imshow(I);
    [x,y] = ginput(2);

    % Create the first state from the initialization.
    c = [x(1); y(1)];
    r = sqrt( (x(1)-x(2))^2 + (y(1)-y(2))^2 );
    init_state = [c; r; 0; 0; 0];
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
TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.lkt_pos = zeros(n, 3);
TrackedObject.jt_pos = zeros(n, 3);
TrackedObject.pos = zeros(n, 3);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 3);

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
    
%     % If we got some bad data, get rid of it.
%     if abs(TrackedObject.lkt_pos(index, 3)-TrackedObject.template_pos(index, 3)) > 1.5*distance_threshold
%         M = TrackedObject.M(:, :, index-1);  % Use the old M.
%     end
            
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
    [circle, ~] = pipeJointTracker(frames(:, :, :, i+1), weights, circle, evaluation);
    toc;
    
    %%%%%%%%%%%%%%%%%%%%%
    % Distance Estimation
    %%%%%%%%%%%%%%%%%%%%%
    % Get the scaling factor to estimate position from LKT
    alpha = M(1, 1);
    lkt_pos = [M(1, 3)/alpha M(2, 3)/alpha (1 - alpha)*510*(1/250)] + ...
               TrackedObject.template_pos(index, :);
    
    % Get the scale ratio to estimate position from joint tracking.
    ratio = pipe_radius/circle.state(3);
    
    % Convert measurements from pixels to deci-feet.
    delta = .1* [(circle.state(1) - size(frames, 2)/2) * ratio ...
                 (circle.state(2) - size(frames, 1)/2) * ratio ...
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
        gamma = 0.8;
    end
    
    % Combine the LKT position estimate and the joint tracking position
    % estimate.
    pos = gamma * lkt_pos + (1-gamma) * jt_pos;
    
    %%%%%%%%%%%%%%%%%%%%%
    % Save Results.
    %%%%%%%%%%%%%%%%%%%%%
    % Add everything to the result
    TrackedObject.M(:, :, index) = M;
    TrackedObject.error{index} = error;
    TrackedObject.lkt_pos(index, :) = lkt_pos;
    TrackedObject.jt_pos(index, :) = jt_pos;
    TrackedObject.pos(index, :) = pos;
    TrackedObject.template{index} = templateData;
    TrackedObject.template_pos(index+1, :) = TrackedObject.template_pos(index, :);
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the template.
    %%%%%%%%%%%%%%%%%%%%%
    % If we have moved past a threshold re-initialize the template.
    if abs(TrackedObject.lkt_pos(index, 3)-TrackedObject.template_pos(index, 3)) > distance_threshold
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(index+1, :) = lkt_pos;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%
    % Updating the circles.
    %%%%%%%%%%%%%%%%%%%%%
    % If we are tracking too big a circle reinitialize it.
    if circle.state(3) > 240
        % Reinitialize the circle.
        circle.state(3) = small_radius_guess;
        circle.state(6) = small_delta_radius_guess;
        circle.sigma(1:3) = [10 10 10];
        circle.real = false;

        % Set the flag to update pos, once the circle is real.
        update_pos = true;
    end
    
    % Update the initial position, once the circle is real.
    if circle.real && update_pos
        initial_pos = pos+delta;
    end
end

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if visualize
    for i = start:start+n-1
        % Extract the image.
        I = preprocessImage(frames(50:end-50,50:end-50,:,i+1));

        % Determine the optimal affine transform.
        M = TrackedObject.M(:, :, i-start+1);
        T = TrackedObject.template{i-start+1}.template;
        T = uint8(reshape(T, size(I, 1), size(I, 2)));

        % Warp the image to fit the template.
        template = warp.doWarp(I, M);

        % Smash it back into an image.
        template = uint8(template);

        % Show the template.
        subplot(2, 1, 1);
        imshow(template);
        title(i);

        % Show deviations from the original template.
        subplot(2, 1, 2);
        imagesc(abs(T - template));

        pause(0.3);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the z-axis motion %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load([pipe_name '_groundtruth.mat']);
figure;
plot(start:start+n-1, TrackedObject.pos(:, 3), 'b');
hold on;
plot(start:start+n-1, TrackedObject.lkt_pos(:, 3), 'm');
plot(start:start+n-1, TrackedObject.jt_pos(:, 3), 'g');
plot((ground_truth - ground_truth(start))/10, 'k')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Clean up environment.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear I M T alpha distance_threshold error i template templateData index ground_truth;