function [ang_vel] = compute_angularvel2(theta, flag)
time = 1/30;
ang_vel(1) = 0;
acc = 0;
for i = 25:length(theta)
    if(theta(i)- theta(i-1) ~= 0)
        ang_vel(i) = (theta(i) - theta(i-1))/time;
        acc = (ang_vel(i) - ang_vel(i-1))/time;
    else
        ang_vel(i) = ang_vel(i-1) + acc * time;
    end
end


end