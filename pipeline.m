%% Initialize the environment
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

start = 1;
n = 50; %(size(frames, 4)-1)

% The template is basically the whole image.
rect = [50 50 size(frames, 2)-50 size(frames, 1)-50];

TrackedObject = zeros(n, 4);
TrackedObject(1, :) = rect;

%% For each image run the LKT
templateData = [];
for i = start:start+n-1
    % Run Lucas-Kanade Tracker 
    [u, v, templateData] = LucasKanade(frames(:, :, :, i), ...
        frames(:, :, :, i+1), ...
        rect, templateData);
    
    % Update rectangle based on motion
    rect = rect + [u v u v];
    
    % Add to result
    TrackedObject(i-start+2, :) = rect;
end

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%% Visualize
[x, y] = meshgrid(1:size(frames, 2), 1:size(frames, 1));
for i = start:start+n-1
    % Extract the image.
    I = double(rgb2gray(frames(:,:,:,i)));
    
    % Interpolate the rectangle coordinates.
    rect = TrackedObject(i, :);
    [xi, yi] = meshgrid(rect(1):rect(3), rect(2):rect(4));
    
    % Get the tracked template.
    template = interp2(x, y, I, xi, yi);
    template = uint8(template);

    % Show the template.
    imshow(template);

    pause(0.1);
end