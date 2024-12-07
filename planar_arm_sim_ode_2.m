function [dq] = planar_arm_sim_ode_2(t,q,u, params)
    dq = planar_arm_dynamics(q,u, params);
end

