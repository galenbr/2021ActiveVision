clear; close all; clc;
c = cube();

% Generate piercing forces
vectors = c.generate_force();
% Run simulations
c.simulator(vectors);
% Find max piercing angle
max = c.max_piercing_angle();
disp(max);