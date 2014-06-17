%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

addpath ./toolbox

% pipe_name = 'pipe2';

% Load the frames to process.
if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

% Setup the email client
setup_mail;

start = 1;
n = size(frames, 4) - start;
visualize = false;

% Constant for the amount of the image we ignore,
border_size = 10;

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
    odom_rect = [min(size(frames, 2)-2*border_size, max(1, x'-border_size)) ...
                 min(size(frames, 1)-2*border_size, max(1, y'-border_size))];
    odom_rect = int16(odom_rect);
    clear x y;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the result struct %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a structure for saving the results.
TrackedObject.pos = zeros(n, 3);
% Time results.
TrackedObject.time = zeros(n, 1);
% LKT Results
TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For each image run the LKT  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = start:start+n-1
    index = i-start + 1;

    tic;
    % Run Lucas-Kanade Tracker 
    [M, templateData, error] = ...
        LucasKanade(frames(border_size:end-border_size, border_size:end-border_size, :, i), ...
                    frames(border_size:end-border_size, border_size:end-border_size, :, i+1), ...
                    M, warp, templateData, odom_rect);
    t = toc;
    fprintf('Elapsed time is %f seconds.\n', t);
    
    if mod(i, 200) == 0
        disp(i);
    end
        
    % Get the scaling factor to estimate position.
    alpha = M(1, 1);
    
    % Add to result
    % Distance results
    TrackedObject.pos(index, :) = [M(1, 3)/alpha M(2, 3)/alpha (1 - alpha)*510*(1/250)];
    % Time results.
    TrackedObject.time(index) = t;
    % LKT Results
    TrackedObject.M(:, :, index) = M;
    TrackedObject.error{index} = error;
    TrackedObject.template{index} = templateData;
    TrackedObject.template_pos(index+1, :) = TrackedObject.template_pos(index, :);
    
    % If we got some bad data, get rid of it.
    if abs(TrackedObject.pos(index, 3)) > 1.5*distance_threshold
        M = TrackedObject.M(:, :, index-1);  % Use the old M.
        TrackedObject.M(:, :, index) = M;
        alpha = M(1, 1);
        TrackedObject.pos(index, :) = [M(1, 3)/alpha M(2, 3)/alpha (1 - alpha)*510*(1/250)];

        % Reinitialize the template.
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(index+1, :) = TrackedObject.pos(index, :) + TrackedObject.template_pos(index, :);
        
    % If we have moved past a threshold re-initialize the template.
    elseif abs(TrackedObject.pos(index, 3)) > distance_threshold
        templateData = [];
        M = eye(3);
        TrackedObject.template_pos(index+1, :) = TrackedObject.pos(index, :) + TrackedObject.template_pos(index, :);
    end
    
end

% Calculate overall position of the robot.
pos = TrackedObject.pos + TrackedObject.template_pos(1:end-1, :);

%% Save off the tracked information
save([pipe_name '_lkt_result.mat'], 'pos');

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
% load([pipe_name '_groundtruth.mat']);
figure;
plot(start:start+n-1, pos(:, 3)*0.3048*0.22, 'b');
% plot(start:start+n-1, pos(:, 3), 'b');
hold on;
plot(start:start+n, TrackedObject.template_pos(:, 3)*0.3048*0.22, 'r')
plot(start:start+n-1, TrackedObject.pos(:, 3)*0.3048*0.22, 'g')
% plot(start:start+n, (ground_truth(start:start+n) - ground_truth(start))/10, 'k')
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
                  'Testing improvements from changing the findDarkRegions function.' ...
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
