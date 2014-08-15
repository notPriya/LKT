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

start = 100;
% n = size(frames, 4) - start;
n=300;
visualize = false;

% Scale factor for the crawler.
scale_factor =  -0.1278;
% Focal length of the camera.
camera_f = 510;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the LKT variables %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The initial assumption is that we havent transformed.
M = eye(3,3);
templateData = [];
distance_threshold = 20;

% Set the kind of warp we are using.
warp = getRigidBodyWarp();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize the result struct %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a structure for saving the results.
TrackedObject.pos = zeros(n, 4);
% Time results.
TrackedObject.time = zeros(n, 1);
% LKT Results
TrackedObject.M = zeros(3, 3, n);
TrackedObject.error = cell(n, 1);
TrackedObject.template = cell(n, 1);
TrackedObject.template_pos = zeros(n+1, 4);

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
    pos = [M(1, 3)/alpha; M(2, 3)/alpha; (alpha - 1)*camera_f; M(1, 2)/alpha*(180/pi)] + ...
           TrackedObject.template_pos(index, :)';
            
    %%%%%%%%%%%%%%%%%%%%%
    % Save Results.
    %%%%%%%%%%%%%%%%%%%%%
    % Add everything to the result
    % Distance results
    TrackedObject.pos(index, :) = pos;
    % Time results.
    TrackedObject.time(index) = t;
    % LKT Results
    TrackedObject.M(:, :, index) = M;
    TrackedObject.error{index} = error;
    TrackedObject.template{index} = templateData;
    TrackedObject.template_pos(index+1, :) = TrackedObject.template_pos(index, :);    
            
    % If we have moved past a threshold re-initialize the template.
    if any(abs(TrackedObject.pos(index, :)-TrackedObject.template_pos(index, :)) > distance_threshold)
        % Clear the template and M.
        templateData = [];
        M = eye(3);
        % Update the template's position.
        TrackedObject.template_pos(index+1, :) = pos;
    end
end

%% Save off the tracked information
pos = TrackedObject.pos;
save([pipe_name '_lkt_results.mat'], 'pos');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if visualize
    mov = VideoWriter('crawler_lkt_results.avi');
    open(mov);
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
%         subplot(2, 1, 1);
        imshow(template);
%         title(i);
        
        % Show deviations from the original template.
%         subplot(2, 1, 2);
%         imagesc(abs(T - template));

%         pause(0.3);
        writeVideo(mov, gcf);
        close;
    end
    close(mov);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visualize the robot motion  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
hold on;
% plot(start:start+n-1, scale_factor*1./camera_f*TrackedObject.pos, '--', 'LineWidth', 2);
plot(scale_factor*1./camera_f*TrackedObject.pos(:, 1), scale_factor*1./camera_f*TrackedObject.pos(:, 2), '--', 'LineWidth', 2);
% plot(camera_pos(:, 2), camera_pos(:, 1), 'k', 'LineWidth', 2);

% plot(start:start+n, pose(:, 3) - pose(1, 3), 'b', 'LineWidth', 2);
% plot(start:start+n, pose(:, 1) - pose(1, 1), 'g', 'LineWidth', 2);
% plot(start:start+n, pose(:, 2) - pose(1, 2), 'r', 'LineWidth', 2);

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
