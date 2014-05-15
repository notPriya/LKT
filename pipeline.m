%% Initialize the environment
clc;

addpath ./toolbox

pipe_name = 'pipe1';

if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

start = 1;
n = (size(frames, 4)-1);
% n = 300;
visualize = false;

% The initial assumption is that we havent transformed.
M = eye(3,3);

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

TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = zeros(n, 1);
TrackedObject.pos = zeros(n, 3);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 3);
distance_threshold = .1;

%% For each image run the LKT
template_frame = start;
templateData = [];
for i = start:start+n-1
    index = i-start + 1;
    
    tic;
    % Run Lucas-Kanade Tracker 
    [M, templateData, error] = ...
        LucasKanade(frames(50:end-50, 50:end-50, :, i), ...
                          frames(50:end-50, 50:end-50, :, i+1), ...
                          M, warp, templateData, odom_rect);
    toc;
    
%     % Visualize M since this takes so damn long.
%     I = preprocessImage(frames(50:end-50, 50:end-50, :, template_frame));
%     I2 = preprocessImage(frames(50:end-50, 50:end-50, :, i+1));
%     template = warp.doWarp(I2, M);
%     template = uint8(template);
%     subplot(2, 1, 1);
%     imshow(template);
%     title(i);
%     subplot(2, 1, 2);
%     imagesc(abs(uint8(I) - template));
%     drawnow;
   
    
    % Get the scaling factor to estimate position.
    alpha = M(1, 1);
    
    % Add to result
    TrackedObject.M(:, :, index) = M;
    TrackedObject.error(index) = min(error);
    TrackedObject.pos(index, :) = [M(1, 3)/alpha M(2, 3)/alpha (1-alpha)*510*(1/250)];
    TrackedObject.template{index} = templateData;
    TrackedObject.template_pos(index+1, :) = TrackedObject.template_pos(index, :);
    
    % If we got some bad data, get rid of it.
    if abs(TrackedObject.pos(index, 3)) > 1.5*distance_threshold
        M = TrackedObject.M(:, :, index-1);  % Use the old M.
        TrackedObject.M(:, :, index) = M;
        alpha = M(1, 1);
        meow = [M(1, 3)/alpha M(2, 3)/alpha (1-alpha)*510*(1/250)]
        TrackedObject.pos(index, :) = meow;

        template_frame = i;
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(index+1, :) = TrackedObject.pos(index, :) + TrackedObject.template_pos(index, :);

        
    % If we have moved past a threshold re-initialize the template.
    elseif abs(TrackedObject.pos(index, 3)) > distance_threshold
        template_frame = i;
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(index+1, :) = TrackedObject.pos(index, :) + TrackedObject.template_pos(index, :);
    end
    
end

% Calculate overall position of the robot.
pos = TrackedObject.pos + TrackedObject.template_pos(1:end-1, :);
% plot3(pos(:, 1), pos(:, 2), pos(:, 3), 'k', 'LineWidth', 2);

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%% Visualize
if visualize
    for i = start:start+n-1
%     for i=133
        % Extract the image.
        I = preprocessImage(frames(50:end-50,50:end-50,:,i+1));

        % Determine the optimal affine transform.
        M = TrackedObject.M(:, :, i-start+1);
%         template_frame = TrackedObject.template(i-start+1);
%         T = preprocessImage(frames(50:end-50,50:end-50,:,template_frame));
%         T = uint8(T);
        T = TrackedObject.template{i-start+1}
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

%% Visualize the z-axis motion.
load([pipe_name '_groundtruth.mat']);
figure;
plot(start:start+n-1, pos(:, 3), 'b');
hold on;
plot(start:start+n, TrackedObject.template_pos(:, 3), 'r')
plot(start:start+n-1, TrackedObject.pos(:, 3), 'g')
plot((ground_truth - ground_truth(start))/10, 'k')

%% Clean up environment.

clear I M T alpha distance_threshold error i template templateData x y template_frame index ground_truth;