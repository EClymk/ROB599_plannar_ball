% the dynamics function of the arm ball system
% for both simulation and linearlization

% function dq = lie_group_dynamics(q, u, params)
%     % Lie group dynamics for a planar arm and ball system on SE(2)
%     
%     % Parameters
%     L1 = params.L1;   % Length of link 1 (m)
%     L2 = params.L2;   % Length of link 2 (m)
%     r = params.r;     % Radius of the ball (m)
%     M = params.M;     % Mass of the ball (kg)
%     g = params.g;     % Acceleration due to gravity (m/s^2)
%     I_ball = params.I_ball; % Ball inertia
% 
%     % Extract state variables
%     x = q(1);       dx = q(2);
%     y = q(3);       dy = q(4);
%     Lb = q(5);      dLb = q(6);
%     theta1 = q(7);  dtheta1 = q(8);
%     theta2 = q(9);  dtheta2 = q(10);
%     phi = q(11);    % Ball's orientation
% 
%     % Joint accelerations
%     ddtheta1 = u(1);
%     ddtheta2 = u(2);
% 
%     % Compute kinematic terms
%     beta = theta1 + theta2 - pi;
%     dbeta = dtheta1 + dtheta2;
% 
%     % Jacobians for link accelerations
%     ddx_P1 = -L1 * cos(theta1) * dtheta1^2 - L1 * sin(theta1) * ddtheta1;
%     ddy_P1 = -L1 * sin(theta1) * dtheta1^2 + L1 * cos(theta1) * ddtheta1;
% 
%     % Rolling constraint dynamics for ball
%     ddLb = -g * sin(beta) + ...
%            (I_ball / (M * r^2)) * ...
%            (ddx_P1 * cos(beta) + ddy_P1 * sin(beta) - Lb * dbeta^2);
% 
%     % Angular velocity and twist
%     omega_b = dLb / r; % Angular velocity of the ball
%     vb = [dx; dy; omega_b]; % Linear and angular velocity twist in SE(2)
% 
%     % Twist matrix (Lie algebra element in se(2))
%     xi_b_wedge = [0, -vb(3), vb(1); % Skew-symmetric twist matrix
%                   vb(3), 0, vb(2);
%                   0, 0, 0];
% 
%     % Construct current pose in SE(2)
%     g_b = [cos(phi), -sin(phi), x;  % Ball's position and orientation
%            sin(phi), cos(phi), y;
%            0, 0, 1];
% 
%     % Update pose using matrix exponential (Lie group update)
% %     dt = params.dt; % Time step
% %     g_b_new = g_b * expm(dt * xi_b_wedge);
% % 
% %     % Extract updated pose components
% %     x_new = g_b_new(1, 3);
% %     y_new = g_b_new(2, 3);
% %     phi_new = atan2(g_b_new(2, 1), g_b_new(1, 1)); % Orientation from rotation matrix
% 
%     % Assemble state derivatives
%     dq = [dx; ...               % x velocity
%           ddLb * cos(beta); ... % x acceleration
%           dy; ...               % y velocity
%           ddLb * sin(beta); ... % y acceleration
%           dLb; ...              % Lb velocity
%           ddLb; ...             % Lb acceleration
%           dtheta1; ...          % theta1 velocity
%           ddtheta1; ...         % theta1 acceleration
%           dtheta2; ...          % theta2 velocity
%           ddtheta2; ...         % theta2 acceleration
%           omega_b];             % Angular velocity (dphi)
% end


function dq = lie_group_dynamics(q, u, params)
    % Lie group dynamics for planar arm and ball system with 11 variables
    
    % Parameters
    L1 = params.L1;   % Length of link 1 (m)
    L2 = params.L2;   % Length of link 2 (m)
    r = params.r;     % Radius of the ball (m)
    M = params.M;     % Mass of the ball (kg)
    g = params.g;     % Acceleration due to gravity (m/s^2)
    I_ball = params.I_ball; % Ball inertia

    % Extract state variables
    x = q(1);       dx = q(2);
    y = q(3);       dy = q(4);
    Lb = q(5);      dLb = q(6);
    theta1 = q(7);  dtheta1 = q(8);
    theta2 = q(9);  dtheta2 = q(10);
    phi = q(11);    % Ball's orientation

    % Joint accelerations
    ddtheta1 = u(1);
    ddtheta2 = u(2);

    % Kinematics: Compute beta terms
    beta = theta1 + theta2 - pi;
    dbeta = dtheta1 + dtheta2;

    % Jacobians for positions and velocities
    ddx_P1 = -L1 * cos(theta1) * dtheta1^2 - L1 * sin(theta1) * ddtheta1;
    ddy_P1 = -L1 * sin(theta1) * dtheta1^2 + L1 * cos(theta1) * ddtheta1;

    % Dynamics of the ball (rolling constraint)
    ddLb = -g * sin(beta) + ...
           (I_ball / (M * r^2)) * ...
           (ddx_P1 * cos(beta) + ddy_P1 * sin(beta) - Lb * dbeta^2);

    % Ball acceleration
    ddx = ddLb * cos(beta) + ddx_P1;
    ddy = ddLb * sin(beta) + ddy_P1;

    % Angular velocity of the ball (from rolling constraint)
    omega_b = dLb / r;

    % Construct the twist (Lie algebra element) for the ball
    vb = [dx; dy; omega_b];  % Ball's twist in SE(2)
    twist_se2 = [0, -vb(3), vb(1);  % Skew-symmetric matrix of twist
                 vb(3), 0, vb(2);
                 0, 0, 0];

    % Update the pose (g_b) of the ball
    g_b = [cos(phi), -sin(phi), x;  % Ball's position and orientation
           sin(phi), cos(phi), y;
           0, 0, 1];  % Homogeneous transform

    % Compute the derivative of the pose (g_b)
    dg_b = g_b * twist_se2;

    % Update the ball's position and velocity (dx, dy)
    dx_new = dg_b(1, 3);  % Change in x position
    dy_new = dg_b(2, 3);  % Change in y position

    % Combine dynamics for all states
    dq = [dx_new; ddx; dy_new; ddy; dLb; ddLb; dtheta1; ddtheta1; dtheta2; ddtheta2; omega_b];
end