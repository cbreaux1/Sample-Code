function [dxdt] = Linear_LQR_Model(A, B, C, D, G, x)
    % LQR input
    u = -G*x;
    % output xdot
    dxdt = A*x + B*u;
end