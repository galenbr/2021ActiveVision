% Initialize start pose
th1_start = 40;
th2_start = -35;

%Initialize goal pose
th1_goal = 55;
th2_goal = -10;

% Initial Jacobian
% Can either be zeros or an approximated jacobian for the starting position
Q = zeros(2,2)

% Estimation variables
cur_pos = [th1_start, th2_start];
goal_pos = [th1_goal, th2_goal];
th1_ = th1_start;
th2_ = th2_start;

%% Control Loop
while(err > 0.1)
    % Compute error
    err = compute_error(cur_pos, goal_pos);
    
    % End effector Position
    [ee_pos, j2_pos] = compute_pos(cur_pos);
    
    
    ds(t,:) = shape_change(pos, old_pos);
    old_pos = pos;
    dr(t,:) = angle_change(r, old_r);
    old_r = r;
end