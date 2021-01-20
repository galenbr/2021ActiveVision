clear;
%% Reading data from csv
ee_marker1 = csvread('data/exp4/px3.csv',2,0);
ee_marker2 = csvread('data/exp5/px3.csv',2,0);
ee_marker3 = csvread('data/exp6/px3.csv',2,0);
ee_marker4 = csvread('data/exp7/px3.csv',2,0);

% End effector position plot

subplot(2,2,1)
plot(ee_marker1(:,2),-ee_marker1(:,3),'.')
title('1')

subplot(2,2,2)
plot(ee_marker2(:,2),-ee_marker2(:,3),'.')
title('2')

subplot(2,2,3)
plot(ee_marker3(:,2),-ee_marker3(:,3),'.')
title('3')

subplot(2,2,4)
plot(ee_marker4(:,2),-ee_marker4(:,3),'.')
title('4')