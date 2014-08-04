% close all;
% clear frames init_state odom_rect TrackedObject;

pipe_name = 'trial8';
% load(['vc_' pipe_name '_data2.mat']);
% frames = recordLog.feedback.video;

pipeline;

close all;
clear frames init_state odom_rect TrackedObject;

pipe_name = 'trial9';
load(['vc_' pipe_name '_data2.mat']);
frames = recordLog.feedback.video;

pipeline;
