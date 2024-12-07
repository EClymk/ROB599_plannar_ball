function q_final = integrate_lie_group_dynamics(q0, u, Ts, params, dt)
    % Integrates the Lie group dynamics from t = 0 to t = Ts
    %
    % Inputs:
    %   q0 - Initial state vector (11x1)
    %   u - Control inputs [u1, u2] (joint accelerations)
    %   Ts - Final simulation time (seconds)
    %   params - Structure with system parameters
    %   dt - Time step for integration (seconds)
    %
    % Output:
    %   q_final - State vector at t = Ts

    % Initialize state
    q = q0;
    t = 0;

    % Time-stepping loop
    while t < Ts
        % Ensure we don't overshoot the final time
        if t + dt > Ts
            dt = Ts - t;
        end
        
        % Update state using the provided dynamics function
        q = lie_group_dynamics_update(q, u, dt, params);
        
        % Increment time
        t = t + dt;
    end

    % Final state
    q_final = q;
end