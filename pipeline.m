%% Initialize the environment
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

start = 1;
n = 200; %(size(frames, 4)-1)

% The initial assumption is that we havent transformed.
M = eye(3,3);

% Set the kind of warp we are using.
warpFn = @affineWarp;

TrackedObject = zeros(3, 3, n);

%% For each image run the LKT
templateData = [];
for i = start:start+n-1
    % Run Lucas-Kanade Tracker 
    [M, templateData, error] = ...
        LucasKanadeAffine(frames(50:end-50, 50:end-50, :, i), ...
                          frames(50:end-50, 50:end-50, :, i+1), ...
                          M, warpFn, templateData);
    
    % Add to result
    TrackedObject(:, :, i-start+1) = M;
end

%% Save off the tracked information
% save('trackCoordinates_real.mat', 'TrackedObject');

%% Visualize
T = rgb2gray(frames(50:end-50,50:end-50,:,start));
[x, y] = meshgrid(1:size(T, 2), 1:size(T, 1));
for i = start:start+n-1
    % Extract the image.
    I = double(rgb2gray(frames(50:end-50,50:end-50,:,i+1)));

    % Determine the optimal affine transform.
    M = TrackedObject(:, :, i-start+1);
    
    % Warp the image to fit the template.
    template = warpFn(I, M);

    % Smash it back into an image.
    template = uint8(template);

    % Show the template.
    subplot(2, 1, 1);
    imshow(template);
    
    % Show deviations from the original template.
    subplot(2, 1, 2);
    imagesc(T - template);

    pause(0.1);
end