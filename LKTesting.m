%% Tests for the Lucas Kanade Affine code.
clc; 

%% Initialize the environment.
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

I = frames(50:end-50, 50:end-50, :, 1);

warpFn = @affineWarp;

threshold = .1;

%% Test the trivial cases.

% Set up the identity warp.
A = eye(3);
tform = affine2d(A);

% Transform the image.
I2 = imwarp(I, tform, 'OutputView', imref2d(size(I)));

% Run Lucas Kanade.
M = LucasKanade(I2, I, A', warpFn, []);

% Show results of test.
if ~all(all(M == A'))
   disp('Identity Test Failed'); 
end

%% Test the non-trivial cases.

% Setup some non-trivial warp.
B = A + [.01*randn(3,2) zeros(3, 1)];
tform2 = affine2d(B);

% Transform the image.
I2 = imwarp(I, tform2, 'OutputView', imref2d(size(I)));

M = LucasKanade(I2, I, B', warpFn, []);

if sum(sum(abs(M - B'))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

M = LucasKanade(I2, I, eye(3), warpFn, []);

if sum(sum(abs(M - B'))) > threshold
   disp('Non-trivial Hard Test Failed');     
end

%% Test a more difficult case.

% Setup an even harder warp.
C = A + [.1*randn(3,2) zeros(3, 1)];
tform3 = affine2d(C);

% Transform the image.
I2 = imwarp(I, tform3, 'OutputView', imref2d(size(I)));

M = LucasKanade(I2, I, C', warpFn, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Easy Test Failed');     
end

M = LucasKanade(I2, I, eye(3), warpFn, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Hard Test Failed');     
end

M = LucasKanade(I2, I, eye(3) + [.1*randn(2,3); zeros(1, 3)], warpFn, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Harder Test Failed');     
end

%% Clean up Environment.
disp('Testing Complete');

clear A B C tform tform2 tform3 M I I2 threshold;