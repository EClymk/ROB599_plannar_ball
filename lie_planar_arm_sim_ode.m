
function [dq] = lie_planar_arm_sim_ode(t,q,u, params)
    dq = lie_group_dynamics(q,u, params);
end

