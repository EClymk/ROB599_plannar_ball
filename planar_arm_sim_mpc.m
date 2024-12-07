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
T = 1;
% state thould satisfy the geometry constriait

% theta1_0 = pi/4;   % Initial angle of link 1 (rad)
% theta2_0 = 3 * pi/4;   % Initial angle of link 2 (rad)
% Lb_0 = 0.5;
% x_0 = params.L1 * cos(theta1_0) + Lb_0 * cos(theta1_0+theta2_0-pi) - params.r * sin(theta1_0+theta2_0-pi);
% y_0 = params.L1 * sin(theta1_0) + Lb_0 * sin(theta1_0+theta2_0-pi) + params.r * cos(theta1_0+theta2_0-pi);

x_0 = 1.6;
y_0 = 0.3;

[theta1_0, theta2_0, Lb_0] = get_thetas_xy(x_0, y_0, params);


x_des = 1.6;%x_0-0.01;
y_des = -0.3;%y_0-0.03;
[theta1_des, theta2_des, Lb_des] = get_thetas_xy(x_des, y_des, params);

q0 = [x_0 0 y_0 0 Lb_0 0 theta1_0 0 theta2_0 0].';

qdes = [x_des 0 y_des 0 Lb_des 0 theta1_des 0 theta2_des 0].';

R = 2*eye(2);
%Q2_c cost:
Q = eye(10);
% Q = diag([8,8,8,8,8,8,3,3,3,3]);
% Q = diag([200,1,200,1,200,1,0.1,1.5,0.1,1.5]);


N = 21;
Torquemax = 50;
Thrustmax = 300;
ddthetamax = 1.2;
Tfinal = 10;
Ts = 0.2;
t_sim = 0:Ts:Tfinal;
t_all = [];
q_all = [];
u_all = [];
fval_all = [];

tho_vec = q0*ones(1, N);
uo_vec = [0;0]*ones(1,N);
for iter = 1:numel(t_sim)
    % Step 1: Sample the current state
    % It's q0
    % Step 2: Formulate and Run the trajectory optimization
    % Define our Matrices/vectors for a QP
    H = Hfunc(Q,R,qdes,T);
    c = cfunc(Q,R,qdes,T);
    A = Afunc(T, ddthetamax);
    b = bfunc(T, ddthetamax);
    Aeq = Aeqfunc(q0,qdes,tho_vec,T);
    beq = beqfunc(q0,qdes,tho_vec,T);
    [xstar,fval] = quadprog(H,c,A,b,Aeq,beq);
    % Step 3: Apply the optimal control inputs for Ts seconds

    u = [xstar(1:2:2*N),xstar(2:2:N*2)]; % input over time
    u_real = u;

    odefunparams = @(t,q) planar_arm_sim_ode_2(t,q,[interp1(linspace(0,T,N),u_real,t)], params);
    [tout, qout] = ode45(odefunparams,[0 Ts],q0);
    t_all = [t_all; tout+t_sim(iter)];
    q_all = [q_all; qout];

    u_ = interp1(linspace(0,T,N),u,tout);
    u_all = [u_all; u_];
    fval_all = [fval_all, fval];
    % NOW Sample the current state:
    q0 = qout(end,:).';


    tho_vec= q0*ones(1, N);
    
end
vars={'x', 'dx', 'y', 'dy', 'Lb', 'dLb','\theta1', 'd\theta1', '\theta2','d\theta2', 'dd\theta1', 'dd\theta2'};

stackedplot(t_all, [q_all,u_all],"DisplayLabels", vars,"Title", 'Goal: [1.1,0.2]')

planar_ball_animation(15, 'video_1', t_all, q_all, [x_des, y_des])

