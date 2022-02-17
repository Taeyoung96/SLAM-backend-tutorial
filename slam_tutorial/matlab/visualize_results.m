clear; close all; clc;

%% show initial poses
init_data = importdata('../data/initial_poses.csv');

subplot(1,2,1);
plot_poses(init_data);


%% show optimized poses
opt_data = importdata('../data/optimized_poses.csv');

subplot(1,2,2);
plot_poses(opt_data);

