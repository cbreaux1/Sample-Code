function phi = phieq(M, g0)
    % Solve for corner balance equilibrium position
    syms x2 x3
    g = g0*[sin(x2); -sin(x3)*cos(x2); -cos(x3)*cos(x2)];
    sol = solve(M*g==0, x2, x3);
    phi = [0; sol.x2(1); sol.x3(1)];
end