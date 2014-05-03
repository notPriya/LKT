%% Initialize the environment
clc;

addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

start = 1;
n = (size(frames, 4)-1);

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
end

TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.pos = zeros(n, 3);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 3);
distance_threshold = .2;

%% For each image run the LKT
templateData = [];
for i = start:start+n-1
    tic;
    % Run Lucas-Kanade Tracker 
    [M, templateData, error] = ...
        LucasKanade(frames(50:end-50, 50:end-50, :, i), ...
                    frames(50:end-50, 50:end-50, :, i+1), ...
                    eye(3), warp, templateData, odom_rect);
    toc;
    
%     % Plot the error function.
%     plot(error);
%     drawnow;
    
    % Get the scaling factor to estimate position.
    alpha = M(1, 1);
    
    % Add to result
    TrackedObject.M(:, :, i-start+1) = M;
    TrackedObject.error{i} = error;
    TrackedObject.pos(i, :) = [M(1, 3)/alpha M(2, 3)/alpha (1 - alpha)*510*(1/250)];
    TrackedObject.template{i} = templateData;
    TrackedObject.template_pos(i+1, :) = TrackedObject.template_pos(i, :);
    
    % If we have moved past a threshold re-initialize the template.
    if abs(TrackedObject.pos(i, 3)) > distance_threshold
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(i+1, :) = TrackedObject.pos(i, :) + TrackedObject.template_pos(i, :);
    end
    
end

% Calculate overall position of the robot.
pos = TrackedObject.pos + TrackedObject.template_pos(1:end-1, :);
% plot3(pos(:, 1), pos(:, 2), pos(:, 3), 'k', 'LineWidth', 2);

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%% Visualize
for i = 1340:1580
    % Extract the image.
    I = preprocessImage(frames(50:end-50,50:end-50,:,i+1));
    [x, y] = meshgrid(1:size(I, 2), 1:size(I, 1));

    % Determine the optimal affine transform.
    M = TrackedObject.M(:, :, i-start+1);
    T = TrackedObject.template{i}.template;
    T = uint8(reshape(T, size(I, 1), size(I, 2)));
    
    % Warp the image to fit the template.
    template = warp.doWarp(I, M);

    % Smash it back into an image.
    template = uint8(template);

    % Show the template.
    subplot(2, 1, 1);
    imshow(template);
    
    % Show deviations from the original template.
    subplot(2, 1, 2);
    imagesc(T - template);

    pause(0.3);
end

%% Visualize the z-axis motion.

figure;
plot(pos(:, 3), 'b');
hold on;
plot(TrackedObject.template_pos(:, 3), 'r')
plot(TrackedObject.pos(:, 3), 'g')

%% Clean up environment.

clear I M T alpha distance_threshold error i template templateData x y;