clc
clear
close all


% Parameters
params = struct;
params.L1 = 1;   % Length of link 1 (m)
params.L2 = 1;   % Length of link 2 (m)
params.r = 0.1;  % Radius of the ball (m)
params.M = 1;    % Mass of the ball (kg)
params.g = 9.8;  % Acceleration due to gravity (m/s^2)
params.I_ball = 2/3 * params.M * params.r^2; % inertia o fthe ball (kg*m^2) 


% Run a quick test optimization
T = 2;
% state thould satisfy the geometry constriait

theta1 = pi/4;   % Initial angle of link 1 (rad)
theta2 = 3 * pi/4-0.002;   % Initial angle of link 2 (rad)
Lb_0 = 0.5;
x_0 = params.L1 * cos(theta1) + Lb_0 * cos(theta1+ theta2-pi) - params.r * sin(theta1+ theta2-pi);
y_0 = params.L1 * sin(theta1) + Lb_0 * sin(theta1+ theta2-pi) + params.r * cos(theta1+ theta2-pi);

x_des = x_0-0.1;
y_des = y_0-0.05;
[theta1_des, theta2_des, Lb_des] = get_thetas_xy(x_des, y_des, params);

q0 = [x_0 0 y_0 0 Lb_0 0 theta1 0 theta2 0].';
% qdes = [x_des 0 y_des 0 Lb_des 0 theta1_des 0 theta2_des 0].';
qdes = [x_0 0 y_0 0 Lb_0 0 theta1 0 theta2 0].';
R = 0.1*eye(2);
%Q2_c cost:
Q = eye(10);
% Q = diag([1,1,1,1,0,0,0,0]);


N = 21;
Torquemax = 50;
Thrustmax = 300;
Tfinal = 20;
Ts = 20;
t_sim = 0:Ts:Tfinal;
t_all = [];
q_all = [];
u_all = [];
fval_all = [];


odefunparams = @(t,q) planar_arm_sim_ode_2(t,q,[0;0], params);
[tout, qout] = ode45(odefunparams,[0 Ts],q0);
t_all = tout;
q_all = qout;


% NOW Sample the current state:
q0 = qout(end,:).';


    

% vars={'x', 'dx', 'y', 'dy', '\theta', 'd\theta', '\tau', 'F_T'};
% stackedplot(t_all, [q_all,u_all],"DisplayLabels", vars)
stackedplot(t_all, q_all)
title('Q3 b')

% TEST energy conservation?
% Ek = 0.5*params.M*(q_all(:, 2).*q_all(:,2)+q_all(:, 4).*q_all(:,4))+...
%     0.5*params.I_ball*(q_all(:, 2).*q_all(:,2)+q_all(:, 4).*q_all(:,4))/(params.r)^2;
% Ek_v = 0.5*params.M*(q_all(:, 2).*q_all(:,2)+q_all(:, 4).*q_all(:,4));
% Ek_r = 0.5*params.I_ball*(q_all(:, 2).*q_all(:,2)+q_all(:, 4).*q_all(:,4))/(params.r)^2;
% Ep = params.M*params.g.*(q_all(:,3)-q_all(1,3));
% E = Ek+Ep;
% figure
% plot(E)

