function [f, coeffs] = fit_curve(curvePtsX, curvePtsY)
    f=fit(transpose(curvePtsX),transpose(curvePtsY),'poly3');
    
    % curve co-efficients
    coeffs = [f.p1, f.p2, f.p3, f.p4];
end