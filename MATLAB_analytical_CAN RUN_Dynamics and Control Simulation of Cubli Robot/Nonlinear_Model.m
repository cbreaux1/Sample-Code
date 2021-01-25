function dxdt = Nonlinear_Model(I, Ihat, Iw, M, g0, Km, Cw, x, u)
    % gravity components
    g = g0*[sin(x(2)); -sin(x(3))*cos(x(2)); -cos(x(3))*cos(x(2))];
    % kinematic equation
    F = [0, sin(x(3))/cos(x(2)),           cos(x(3))/cos(x(2));
          0, cos(x(3)),                     -sin(x(3));
          1, sin(x(3))*sin(x(2))/cos(x(2)), cos(x(3))*sin(x(2))/cos(x(2))];
    % derivatives
    wh = F*x(4:6);
    whd = Ihat^-1*(cross(I*x(4:6),x(4:6)) + M*g + Iw*x(7:9) - (Km*u-Cw*x(7:9)));
    wwd = Iw^-1*(Km*u - Cw*x(7:9) - Iw*whd);
    % output xdot
    dxdt = [wh; whd; wwd];
end