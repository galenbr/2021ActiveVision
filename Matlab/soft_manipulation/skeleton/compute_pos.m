function [ee_pos, j2_pos] = compute_pos(cur_pos)
%% Robot Model 
l1 = 90; l2 = 75;
syms th1 th2;

eeXex = l1*cosd(th1) + l2*cosd(th1 + th2);
eeYex = l1*sind(th1) + l2*sind(th1 + th2);

j2Xex = l1*cosd(th1);
j2Yex = l1*sind(th1);

baseX = 0;
baseY = 0;

% End effector Pose
th1_ = cur_pos(1);
th2_ = cur_pos(2);

eeX = eval(subs(eeXex, [th1, th2], [th1_, th2_]));
eeY = eval(subs(eeYex, [th1, th2], [th1_, th2_]));
j2X = eval(subs(j2Xex, th1, th1_));
j2Y = eval(subs(j2Yex, th1, th1_));

ee_pos = [eeX, eeY];
j2_pos = [j2X, j2Y];


end