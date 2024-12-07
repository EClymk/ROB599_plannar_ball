function [tout, uout] = planar_arm_sim_ode(q0, Tf, params)
% takes in the decision vector theta
% simulate the motion of te block with force defined by theta
% decision vector dtheta: [dtheta_1,dtheta_2], where dtheta_1=[dtheta_1_t1, ...
% dtheta_1_tn]'
% Tf is time span
% y = [x, dx, y, dy, Lb, dLb, theta_1, dtheta_1, theta_2, dtheta_2]'

% assume theta1, theta2, xBall, yBall at time 0 is fixed

% N = numel(dtheta) / 2;
% Parameters

odefun = @(t,q) planar_arm_sim_ode_2(t,q,u, params);

initial_guess = q0;

[tout, uout] = ode45(odefun, [0,Tf], initial_guess);




end
