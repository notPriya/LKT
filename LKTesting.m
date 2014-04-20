%% Tests for the Lucas Kanade Affine code.
clc; 

%% Initialize the environment.
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

I = frames(50:end-50, 50:end-50, :, 1);

threshold = .1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test the Affine Warps.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warp = getAffineWarp();
disp('Starting Affine Warp Tests');

% Test the trivial cases.

% Set up the identity warp.
A = eye(3);
tform = affine2d(A);

% Transform the image.
I2 = imwarp(I, tform, 'OutputView', imref2d(size(I)));

% Run Lucas Kanade.
M = LucasKanade(I2, I, A', warp, []);

% Show results of test.
if ~all(all(M == A'))
   disp('Identity Test Failed'); 
end

% Test the non-trivial cases.

% Setup some non-trivial warp.
B = A + [.01*randn(3,2) zeros(3, 1)];
tform2 = affine2d(B);

% Transform the image.
I2 = imwarp(I, tform2, 'OutputView', imref2d(size(I)));

M = LucasKanade(I2, I, B', warp, []);

if sum(sum(abs(M - B'))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

M = LucasKanade(I2, I, eye(3), warp, []);

if sum(sum(abs(M - B'))) > threshold
   disp('Non-trivial Hard Test Failed');     
end

% Test a more difficult case.

% Setup an even harder warp.
C = A + [.1*randn(3,2) zeros(3, 1)];
tform3 = affine2d(C);

% Transform the image.
I2 = imwarp(I, tform3, 'OutputView', imref2d(size(I)));

M = LucasKanade(I2, I, C', warp, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Easy Test Failed');     
end

M = LucasKanade(I2, I, eye(3), warp, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Hard Test Failed');     
end

M = LucasKanade(I2, I, eye(3) + [.1*randn(2,3); zeros(1, 3)], warp, []);

if sum(sum(abs(M - C'))) > threshold
   disp('Difficult Harder Test Failed');     
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test the Rigid Body Warps.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warp = getRigidBodyWarp();
disp('Starting Rigid Body Warp Tests');

% Test the trivial cases.

% Set up the identity warp.
A = eye(3);
tform = affine2d(A);

% Transform the image.
I2 = imwarp(I, tform, 'OutputView', imref2d(size(I)));

% Run Lucas Kanade.
M = LucasKanade(I2, I, A, warp, []);

% Show results of test.
if ~all(all(M == A))
   disp('Identity Test Failed'); 
end

% Test the non-trivial cases.

% Setup some non-trivial warp.
p = .1*randn(4, 1);
B = warp.newWarp(p);
tform2 = affine2d(B');

% Transform the image.
I2 = imwarp(I, tform2, 'OutputView', imref2d(size(I)));
imshow(I2);
drawnow;

[M, ~, error] = LucasKanade(I2, I, B, warp, []);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

[M, ~, error] = LucasKanade(I2, I, eye(3), warp, []);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Hard Test Failed');     
end

badwarp = warp.newWarp(.1*randn(4, 1));

[M, ~, error] = LucasKanade(I2, I, badwarp, warp, []);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Harder Test Failed');     
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test the Motion Estimation.  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

warp = getRigidBodyWarp();
M = eye(3);
templateData = [];

%%
I1 = frames(50:end-50, 50:end-50, :, 150);
I2 = frames(50:end-50, 50:end-50, :, 200);

[M, templateData, error] = LucasKanade(I1, I2, M, warp, templateData);

alpha = M(1, 1)
theta = atan2(M(2, 1)/alpha, 1);
theta = theta*180/pi
u = M(1, 3)/alpha
v = M(2, 3)/alpha

d = (1 - alpha)*510*(1/250)

figure;
subplot(2, 2, 1);
imshow(I1);
subplot(2, 2, 2);
imshow(I2);
subplot(2, 2, 3);
meow = warp.doWarp(I2, M);
imshow(meow);
subplot(2, 2, 4);
imshow(I1 - meow);

%% Clean up Environment.
disp('Testing Complete');

clear A B C warp tform tform2 tform3 M I I2 threshold error I1 templateData alpha theta u v d meow;