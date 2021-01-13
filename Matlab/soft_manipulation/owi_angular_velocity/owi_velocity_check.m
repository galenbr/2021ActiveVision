%% Reading data from csv
base_marker = csvread('',2,0);
ee_marker = csvread('',2,0);
flag = csvread('',2,0);
input_vel = csvread('',2,0);

%% Calculating joint angle
theta = compute_angle(base_marker,ee_marker);

%% Calculating angular velocity
% Case 1: no acceleration for dropped frames
ang_vel1 = compute_angularvel1(theta,flag);
% Case 2: assuming prev acceleration for dropped frames
ang_vel2 = compute_angularvel2(theta,flag);