%% Tests for the Lucas Kanade Affine code.
clc; 

%% Initialize the environment.
addpath ./toolbox

if ~exist('frames', 'var')
    load 'pipe1_clean.mat';
end

I = frames(50:end-50, 50:end-50, :, 1);
odom_rect = [1 1 1 1];

threshold = .1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test the Affine Warps.       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warp = getAffineWarp();
disp('Starting Affine Warp Tests');

% Test the trivial cases.

% Set up the identity warp.
A = eye(3);

% Transform the image.
I2 = warp.doWarp(I, A);

% Run Lucas Kanade.
M = LucasKanade(I2, I, A, warp, [], odom_rect);

% Show results of test.
if ~all(all(M == A))
   disp('Identity Test Failed'); 
end

% Test the non-trivial cases.

% Setup some non-trivial warp.
B = A * warp.newWarp(.01*randn(6,1));

% Transform the image.
I2 = warp.doWarp(I, B);

M = LucasKanade(I2, I, B, warp, [], odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

M = LucasKanade(I2, I, eye(3), warp, [], odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Hard Test Failed');     
end

% Test a more difficult case.

% Setup an even harder warp.
C = A * warp.newWarp(.1*randn(6,1));

% Transform the image.
I2 = warp.doWarp(I, C);

M = LucasKanade(I2, I, C, warp, [], odom_rect);

if sum(sum(abs(M - C))) > threshold
   disp('Difficult Easy Test Failed');     
end

M = LucasKanade(I2, I, eye(3), warp, [], odom_rect);

if sum(sum(abs(M - C))) > threshold
   disp('Difficult Hard Test Failed');     
end

M = LucasKanade(I2, I, eye(3) + [.1*randn(2,3); zeros(1, 3)], warp, [], odom_rect);

if sum(sum(abs(M - C))) > threshold
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

% Transform the image.
I2 = warp.doWarp(I, A);

% Run Lucas Kanade.
M = LucasKanade(I2, I, A, warp, [], odom_rect);

% Show results of test.
if ~all(all(M == A))
   disp('Identity Test Failed'); 
end

% Test the non-trivial cases.

% Setup some non-trivial warp.
p = [10*randn(2, 1); .1*randn(1); .1*randn(1)];
B = warp.newWarp(p);

% Transform the image.
I2 = warp.doWarp(I, B);

[M, ~, error] = LucasKanade(I2, I, B, warp, [], odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

[M, ~, error] = LucasKanade(I2, I, eye(3), warp, [], odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Hard Test Failed');     
end

badwarp = warp.newWarp(.1*randn(4, 1));

[M, ~, error] = LucasKanade(I2, I, badwarp, warp, [], odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Harder Test Failed');     
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test the NonLinear Warps.   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
warp = getNonLinWarp();
disp('Starting NonLinear Warp Tests');

% Test the trivial cases.

% Set up the identity warp.
p = [0; 0; 1; 0];
A = warp.newWarp(p);

% Transform the image.
I2 = warp.doWarp(I, A);

% Run Lucas Kanade.
M = LucasKanadeNonLin(I2, I, A, warp, odom_rect);

% Show results of test.
if sum(sum(abs(M - A))) > threshold
   disp('Identity Test Failed'); 
end

% Test the non-trivial cases.

% Setup some non-trivial warp.
p = .1*randn(4, 1) + [0; 0; 1; 0];
B = warp.newWarp(p);

% Transform the image.
I2 = warp.doWarp(I, B);
imshow(I2);
drawnow;

[M, error] = LucasKanadeNonLin(I2, I, B, warp, odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Easy Test Failed');  
end

% Perturb the warp.
p2 = p + .05*randn(4, 1);
B2 = warp.newWarp(p2);
[M, error] = LucasKanadeNonLin(I2, I, B2, warp, odom_rect);

if sum(sum(abs(M - B))) > threshold
   disp('Non-trivial Hard Test Failed');     
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