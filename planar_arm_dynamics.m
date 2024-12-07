% the dynamics function of the arm ball system
% for both simulation and linearlization

function dq = planar_arm_dynamics(q, u, params)
    % given current q and u, return the dq
    % q = [x, dx, y, dy, Lb, dLb, theta_1, dtheta_1, theta_2, dtheta_2]'  omegas already in Lb(?)
    % dynamics
    % u = [dtheta_1, ddtheta_1, dtheta_2, ddtheta_2]'
    % params
    L1 = params.L1;   % Length of link 1 (m)
    L2 = params.L2;   % Length of link 2 (m)
    r = params.r;  % Radius of the ball (m)
    M = params.M;    % Mass of the ball (kg)
    g = params.g;  % Acceleration due to gravity (m/s^2)
    I_ball = params.I_ball; % inertia of ball

    % q
    x = q(1);
    dx = q(2);
    y = q(3);
    dy = q(4);
    Lb = q(5);
    dLb = q(6);
    theta_1 = q(7);
    dtheta_1= q(8);
    theta_2 = q(9);
    dtheta_2 = q(10);

    % u
    ddtheta_1 = u(1);
    ddtheta_2 = u(2);

    beta_ = @(the1, the2) the1+the2-pi;
    dbeta = @(dthe1, dthe2) dthe1+dthe2;
    ddbeta = @(ddthe1, ddthe2) ddthe1+ddthe2;
    
    ddx_P1 = @(the1, dthe1, ddthe1) -L1 * cos(the1) * dthe1^2 - L1 * sin(the1) * ddthe1;
    ddy_P1 = @(the1, dthe1, ddthe1) -L1 * sin(the1) * dthe1^2 + L1 * cos(the1) * ddthe1;
    ddLb = @(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb) -g * sin(beta_(the1, the2)) + ...
        I_ball/(M*r^2)*...
        (ddx_P1(the1, dthe1, ddthe1)*cos(beta_(the1, the2))+...
        ddy_P1(the1, dthe1, ddthe1)*sin(beta_(the1, the2))-Lb*ddbeta(ddthe1, ddthe2)^2);

    ddx_P2=@(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)  - Lb * cos(beta_(the1, the2)) * dbeta(dthe1,dthe2)^2 - Lb * sin(beta_(the1, the2)) * ddbeta(ddthe1, ddthe2)...
        - 2 * sin(beta_(the1, the2)) * dLb * dbeta(dthe1,dthe2) ;
    ddy_P2=@(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)  - Lb * sin(beta_(the1, the2)) * dbeta(dthe1,dthe2)^2 + Lb * cos(beta_(the1, the2)) * ddbeta(ddthe1, ddthe2)...
        + 2 * cos(beta_(the1, the2)) * dLb * dbeta(dthe1,dthe2) ;
    ddx = @(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)  ddLb(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)*cos(beta_(the1, the2)) + ddx_P1(the1, dthe1, ddthe1) + ddx_P2(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb);
    ddy = @(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)  ddLb(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb)*sin(beta_(the1, the2)) + ddy_P1(the1, dthe1, ddthe1) + ddy_P2(the1, the2, dthe1, dthe2, ddthe1, ddthe2, Lb);

    dq = [dx, ...
        ddx(theta_1, theta_2, dtheta_1, dtheta_2, ddtheta_1, ddtheta_2, Lb),...
        dy, ...
        ddy(theta_1, theta_2, dtheta_1, dtheta_2, ddtheta_1, ddtheta_2, Lb),...
        dLb, ...
        ddLb(theta_1, theta_2, dtheta_1, dtheta_2, ddtheta_1, ddtheta_2, Lb),...
        dtheta_1,...
        ddtheta_1,...
        dtheta_2,...
        ddtheta_2]';
end