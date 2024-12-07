clc
clear

% Parameters
params = struct;
params.L1 = 1;   % Length of link 1 (m)
params.L2 = 1;   % Length of link 2 (m)
params.r = 0.1;  % Radius of the ball (m)
params.M = 1;    % Mass of the ball (kg)
params.g = 9.8;  % Acceleration due to gravity (m/s^2)
params.I_ball = 2/3 * params.M * params.r^2; % inertia o fthe ball (kg*m^2) 

N = 21;

% single time state:
q = sym('q', [10 1]);
u = sym('u', [2 1]);
% operating points
q0 = sym('q0', [10 1]);
qdes = sym('qdes', [10 1]);


% Dynamics, which should be linearlized at q0(?)
% A_d, B_d stand for dynamics, different from the A,B for qp
A_d_j = jacobian(planar_arm_dynamics(q,u,params),q);
A_d = subs(A_d_j, q, q0);
B_d_j = jacobian(planar_arm_dynamics(q,u,params),u);
B_d = subs(B_d_j, q, q0);
dq = planar_arm_dynamics(q,u,params);

% Define symbolic variables for every state at each node
qN = sym('qN', [10 N]);
uN = sym('uN', [2 N]);
% operating points should be 6-D vector
thoN = sym('thoN', [10 N]);
uoN = sym('uoN', [2 N]);
for i=1:N
    uoN(1, i) = 0;
    uoN(2, i) = 0;
end

% 1st order Dynamics for each of the states at each nodes

dqN = [];
for i=1:N
    Ai = subs(A_d_j, q, thoN(:,i));
    Ai = subs(Ai, u, uoN(:,i));
    Bi = subs(B_d_j, q, thoN(:,i));
    Bi = subs(Bi, u, uoN(:,i));
    dqi = subs(dq, q, thoN(:,i));
    dqi = subs(dqi, u, uoN(:,i));
    % add dynamica at thoN(i)
    dqN = [dqN, Ai*(qN(:,i)-thoN(:,i))+Bi*(uN(:,i)-uoN(:,i))+...
        dqi];
end

% Defect constriants
syms T % constant finite time horizon
dt = T/(N-1);

qNext = qN(:,2:end);
qPrev = qN(:,1:end-1);
dqPrev = dqN(:,1:end-1);
defect = qNext - qPrev - dqPrev*dt;
defect = defect(:);

% test on theta1+theta2=pi
horizon_defect = uN(1,:) + uN(2,:);

% udefect = [];
ddthetamax = sym('ddthetamax', [1 1]); % max torque
% Thrustmax = sym('Thrustmax', [1 1]); % max thrust

% IC constraints
ic_constraint = q0-qN(:,1);
% Inequality
limit_constraints = [uN(1,:).' - ddthetamax;
    -uN(1,:).' - ddthetamax;
    uN(2,:).' - ddthetamax;
    -uN(2,:).' - ddthetamax;
    horizon_defect.' - 0.3;
    -horizon_defect.' - 0.3];

eqCon = [defect; ic_constraint];
ineqCon = limit_constraints;

% Cost Functions

Q = sym('Q', [10 10]);
R = sym('R', [2 2]);


% Define our cost
cost = sum(sum((((qdes-qN(:,1:end-1)).'*Q*(qdes-qN(:,1:end-1)) + ...
uN(:,1:end-1).'*R*uN(:,1:end-1)))*dt));
% Decision vector
x = [uN(:); qN(:)];
% Cost
H = hessian(cost,x);
c = subs(jacobian(cost,x),x,zeros(size(x))).';
% Constraints
A = jacobian(ineqCon,x);
b = -subs(ineqCon,x,zeros(size(x)));
Aeq = jacobian(eqCon,x);
beq = -subs(eqCon,x,zeros(size(x)));
% Export MATLAB Functions
matlabFunction(H,'File','Hfunc','Vars',{Q,R,qdes,T});
matlabFunction(c,'File','cfunc','Vars',{Q,R,qdes,T});
matlabFunction(A,'File','Afunc','Vars',{T,ddthetamax});
matlabFunction(b,'File','bfunc','Vars',{T,ddthetamax});
matlabFunction(Aeq,'File','Aeqfunc','Vars',{q0,qdes,thoN,T});
matlabFunction(beq,'File','beqfunc','Vars',{q0,qdes,thoN,T});


