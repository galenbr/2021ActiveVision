clear; close all; clc;
c = cube();
% Generate piercing forces
vectors = c.generate_force();
% Run simulations
c.simulator(vectors);
%% Max piercing angle trend along various static friction values
u = 0.1:0.05:0.9;
max_p = 1:17;
for i = 1:17;
    max_p(i) = c.max_piercing_angle(u(i));
end

%% Max force trend along various piercing angles
i = 1;
max_f = 1:151;
for p = 15:0.5:90
    max_f(i) = c.max_force(p);
    i = i+1;
end  % for loop
p = 15:0.5:90;
%% Plot Results
figure('Name','Force and Piercing Angle Trends')
subplot(2,1,1)
plot(p,max_f)
xlabel('Piercing Angles')
ylabel('Maximum Force')
legend('u_s: 0.25, mass: 0.05kg')
subplot(2,1,2)
plot(u,max_p)
xlabel('Static Friction Co-efficient')
ylabel('Max Piercing Angle')
legend('F: 1N, mass: 0.05kg')