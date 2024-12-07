N = 10;
u_0 = [zeros(N, 1),zeros(N, 1)];

params = struct;
params.L1 = 1;   % Length of link 1 (m)
params.L2 = 1;   % Length of link 2 (m)
params.r = 0.1;  % Radius of the ball (m)
params.M = 1;    % Mass of the ball (kg)
params.g = 9.8;  % Acceleration due to gravity (m/s^2)
params.I_ball = 2/3 * params.M * params.r^2; % inertia o fthe ball (kg*m^2) 


Tf = 10;

% cost function
cost_step = @(x,t) interp1(linspace(0, Tf, N), x(1:N).', t, "linear").^2;
% cost_step = @(t) 
costfcn = @(x) integral(@(t) cost_step(x(:, 1),t)+cost_step(x(:, 2),t), 0, Tf);

% solve
fmincon_option = optimoptions('fmincon','MaxFunctionEvaluations',1E5);
[uStar, J]= fmincon(costfcn, u_0, [],[],[],[],[],[], @nonlcon_planar_ball, fmincon_option); % Optimal Design Vector
[tout, yout] = planar_arm_sim_ode(uStar, Tf);

figure
plot(tout, yout(:,1))
title('x')
xlabel('time')
ylabel('x/m')

figure
plot(tout, yout(:,3))
title('y')
xlabel('time')
ylabel('y/m')