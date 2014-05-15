function [M, error] = LucasKanadeNonLin(It, It1, M, warp, odom_rect)
    % Initial Guess for parameters and setup optimization.
    p_init = warp.getParam(M);
    opt_fun = @(x)CalculateWarpError(x, It, It1, warp, odom_rect);
    % Linear inequality constraints of form :  A x <= b
    A = [];
    b = [];
    % Linear equality constraints of form :  A x = b
    Aeq = [];
    beq = [];
    % Bounds on the parameters.
    % This is heavily constrained so we dont get bad results
    % from the optimizer. Instead of putting penalty terms in the
    % optimization function making cost more non-linear, we restrict the
    % solution set.
    lb = [-100; -100; .1; -pi/2];
    ub = [100; 100; 5; pi/2];
    
    % Options
    % Throttle max iter since we are close and havent got all day.
    options = optimoptions('fmincon', 'Display', 'off', 'MaxIter', 20);
    
    [p_final, error] = fmincon(opt_fun, p_init, A, b, Aeq, beq, lb, ub, [], options);

    M = warp.newWarp(p_final);
end