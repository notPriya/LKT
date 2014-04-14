%% Initialize the environment
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

start = 1;
n = 50; %(size(frames, 4)-1)

% The initial assumption is that we havent transformed.
M = eye(3,3);

TrackedObject = zeros(3, 3, n);

%% For each image run the LKT
templateData = [];
for i = start:start+n-1
    % Run Lucas-Kanade Tracker 
    [M, templateData] = LucasKanadeAffine(frames(50:end-50, 50:end-50, :, i), ...
        frames(50:end-50, 50:end-50, :, i+1), ...
        M, templateData);
    
    % Add to result
    TrackedObject(:, :, i-start+2) = M;
end

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%% Visualize
[x, y] = meshgrid(1:size(frames, 2), 1:size(frames, 1));
for i = start:start+n-1
    % Extract the image.
    I = double(rgb2gray(frames(:,:,:,i)));

    % Determine the optimal affine transform.
    M = TrackedObject(:, :, i);
    
    % Warp the image to fit the template.
    XiYi = M*[x(:) y(:) ones(length(x(:)), 1)]';
    template = interp2(1:size(I, 2), 1:size(I, 1), I, XiYi(1, :)', XiYi(2, :)', 'linear', 0);
    
    % Smash it back into an image.
    template = reshape(template, size(I, 1), size(I, 2));
    template = uint8(template);

    % Show the template.
    imshow(template);

    pause(0.1);
end