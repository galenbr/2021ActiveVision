%% Reading data from csv
base_marker = csvread('data\exp1\base_px.csv',2,0);
ee_marker = csvread('data/exp1/px3.csv',2,0);
flag = csvread('data/exp1/flag.csv',2,0);
input_vel = csvread('data/exp1/input_vel.csv',2,0);

%% Calculating joint angle
theta = compute_angle(base_marker,ee_marker);

%% Calculating angular velocity
% Case 1: no acceleration for dropped frames
ang_vel1 = compute_angularvel1(theta,flag);
% Case 2: assuming prev acceleration for dropped frames
ang_vel2 = compute_angularvel2(theta,flag);

%% Plot
% Joint angle vs frames
figure('NumberTitle','off','Name','OWI Open Loop Performance')
%title('OWI Open Loop Performance')
subplot(4,1,1)
plot(theta)
title('Joint Angle')
% Input pwm, ang_vel1 vs frames
subplot(4,1,3)
plot(ang_vel1)
title('Measured Angular Velocity')
subplot(4,1,2)
plot(input_vel(:,2))
title('Input PWM')
% Input pwm, ang_vel2 vs frames
subplot(4,1,4)
plot(ang_vel2)
title('Measured Angular Velocity w/ Acc')
% End effector position
figure('NumberTitle','off','Name','End-effector Position')
plot()
% Time plot of end-effector positions and estimated positions for dropped
% frames