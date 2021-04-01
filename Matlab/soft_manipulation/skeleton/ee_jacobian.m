clear
clc

%% Robot Model
l1 = 90; l2 = 75;
% th1 = 35; th2 = -35;
syms th1 th2;
eeXex = l1*cosd(th1) + l2*cosd(th1 + th2);
eeYex = l1*sind(th1) + l2*sind(th1 + th2);
theta1 = [];
theta2 = [];
j2Xex = l1*cosd(th1);
j2Yex = l1*sind(th1);

baseX = 0;
baseY = 0;

%% Estimation Variables
goal_coeffs = zeros(4,1);
old_pos = zeros(2,1);
step = 0.1;
t = 1;
t_end = 30;
th1_start = 40;
th2_start = -30;
th1_ = th1_start;
th2_ = th2_start;
r = zeros(2,1);
old_r = [th1_start; th2_start];
ds = zeros(t_end,2);
dr = zeros(t_end,2);


%% Move Robot locally
disp('Collecting shape data for small movements')
while(t<=t_end)
    eeX = eval(subs(eeXex, [th1, th2], [th1_, th2_]));
    eeY = eval(subs(eeYex, [th1, th2], [th1_, th2_]));
    j2X = eval(subs(j2Xex, th1, th1_));
    j2Y = eval(subs(j2Yex, th1, th1_));
    
    % Sine input velocity
    th1_ = th1_start + sind(step);
    th2_ = th2_start + sind(step);
%     th1_start = th1_start + step;
%     th2_start = th2_start + step;
        
    r = [th1_; th2_];
    step = step + 3;
    theta1 = [theta1, th1_];
    theta2 = [theta2, th2_];
    %% Change in EE Position
    pos = [eeX; eeY];
    
    % initialization step
    if t == 1
        old_pos = pos;
    end
    
    ds(t,:) = shape_change(pos, old_pos);
    old_pos = pos;
    dr(t,:) = angle_change(r, old_r);
    old_r = r;

    
    %% Plots
    figure(1)
    
    % Robot
    plot([0, j2X, eeX], [0, j2Y, eeY], 'b');
    hold on
%     pause(0.1)
    
%     hold off
    %% Collect shape and pose data
    t = t+1;
end
disp('Done collecting data')

% Reset Time
t = 1;

%% Estimate Shape Jacobian
disp('Beginning Shape Jacobian estimation process')

% Inititalize estimation variables
 qhat = zeros(2,2);
%  noise = [100*rand, 40*rand; 10*rand, 30*rand];
%  Q = noise + [-l1*sind(th1_) - l2*sind(th1_+th2_), -l2*sind(th1_+th2_); l1*cosd(th1_) + l2*cosd(th1_+th2_), l2*cos(th1_+th2_)]
%  qhat = Q;
Jplot = [];
while t<=t_end
    [J, qhat_dot] = compute_energy_functional(ds, dr, qhat, t);
    qhat = qhat + qhat_dot;
    Jplot = [Jplot; J];
     t = t+1;
end
figure()
plot(Jplot)
figure()
plot(theta1, 'ro')
figure()
plot(theta2, 'go')