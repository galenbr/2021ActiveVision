function vectors = generate_force()

piercing_angles = [15,30,45,60,90]; %Angles with respect to z-axis

for i = 1:length(piercing_angles)
   
    vec = [sind(piercing_angles(i)), 0 , -cosd(piercing_angles(i))];
    vectors(i,:) = vec;
end

% We will assume these force vectors are acting at the center of the cube
% other wise moments will be generated
% This case can be looked into later but I need a better way to simulate
% the cube