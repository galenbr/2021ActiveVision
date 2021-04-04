function [skelX, skelY, link1PtsX, link1PtsY, link2PtsX, link2PtsY] = skeleton(j2X, j2Y, eeX, eeY)

    % Equation of Link1
    link1PtsX = linspace(0, j2X, 10);
    link1PtsY = linspace(0, j2Y, 10);

    % Equation of Link2
    link2PtsX = linspace(j2X, eeX, 8);
    link2PtsY = linspace(j2Y, eeY, 8);

    skelX = [link1PtsX, link2PtsX(2:end)];
    skelY = [link1PtsY, link2PtsY(2:end)];

end