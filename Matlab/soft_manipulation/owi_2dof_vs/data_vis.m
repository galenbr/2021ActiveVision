clear;
%% Enter goal position
goal = [50, 300];
start = [224, 123];
%% Read data
base_marker = csvread('~/owi_data/vs_data/exp1/base_px.csv',2,0);
shoulder_marker = csvread('~/owi_data/vs_data/exp1/shoulder.csv',2,0);
flag = csvread('~/owi_data/vs_data/exp1/flag.csv',2,0);
input_vel = csvread('~/owi_data/vs_data/exp1/input_vel.csv',2,0);
elbow_marker = csvread('~/owi_data/vs_data/exp1/shoulder.csv',2,0);
ee_marker = csvread('~/owi_data/vs_data/exp1/ee.csv',2,0);

%% Compute error

[err_x,err_y] = compute_error(goal,ee_marker);


%% Plot

figure(1)

subplot(3,1,1)
plot(err_x)

subplot(3,1,2)
plot(err_y)

subplot(3,1,3)
plot(ee_marker(:,2),-ee_marker(:,3),'ro')
hold on
line([start(1),goal(1)],[-start(2),-goal(2)])