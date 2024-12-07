function [C, Ceq] = nonlcon_planar_ball(t,q,u,params)

    Tf = 10;

    [tout, yout] = planar_arm_sim_ode2(t,q,u,params);
    final_state = yout(end,:).';

    Ceq = [];
    % initial
%     Ceq = [Ceq; [x(N+1);x(2*N+1)]];
    % final
    Ceq = [Ceq; [final_state(1)-1.3; final_state(2); final_state(3)-0.6071;final_state(4)]];
    % dynamics
    
    C = [];  

end