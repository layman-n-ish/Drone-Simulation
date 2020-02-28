function [control_signals, params] = resolve_control_signals(controllers, params, desired_state, state)
    thrust = compute_thrust(controllers.thrust_controller, params, desired_state, state.theta, state.x(3), state.xdot(3));
    lateral_acc_cmd = compute_lateral_acc(controllers.x_pos_controller, controllers.y_pos_controller, desired_state, state);
    torques = compute_torques(controllers, params, desired_state, state, lateral_acc_cmd, thrust);
    
    control_signals = zeros(4, 1);
    control_signals(1) = (thrust/(4*params.k)) - (torques(2)/(2*params.k*params.L)) - (torques(3)/(4*params.b));
    control_signals(2) = (thrust/(4*params.k)) - (torques(1)/(2*params.k*params.L)) + (torques(3)/(4*params.b));
    control_signals(3) = (thrust/(4*params.k)) + (torques(2)/(2*params.k*params.L)) - (torques(3)/(4*params.b));
    control_signals(4) = (thrust/(4*params.k)) + (torques(1)/(2*params.k*params.L)) + (torques(3)/(4*params.b));
end

function thrust = compute_thrust(thrust_controller, params, desired_state, theta, z, zdot)
    err_d = thrust_controller.d * (desired_state.xdot(3) - zdot);
    err_p = thrust_controller.p * (desired_state.x(3) - z);
    
    thrust = ((params.g + err_d + err_p) * params.m) / (cos(theta(1)) * cos(theta(2)));
end

function lateral_acc_cmd = compute_lateral_acc(x_pos_controller, y_pos_controller, desired_state, state)
    x_err_d = x_pos_controller.d * (desired_state.xdot(1) - state.xdot(1));
    x_err_p = x_pos_controller.p * (desired_state.x(1) - state.x(1));
    x_acc_cmd = x_err_p + x_err_d;
    
    y_err_d = y_pos_controller.d * (desired_state.xdot(2) - state.xdot(2));
    y_err_p = y_pos_controller.p * (desired_state.x(2) - state.x(2));
    y_acc_cmd = y_err_p + y_err_d;
    
    lateral_acc_cmd = [x_acc_cmd; y_acc_cmd];
end

function torques = compute_torques(controllers, params, desired_state, state, lateral_acc_cmd, thrust)
    torques = zeros(3, 1);
    Izz = params.I(3, 3);
    R = rotation(state.theta);
    tf_mat = (1/R(3, 3)) .* [R(2, 1), -R(1, 1); R(2, 2), -R(1, 2)];
    
    bx_c = (params.m * lateral_acc_cmd(1)) / thrust;
    by_c = (params.m * lateral_acc_cmd(2)) / thrust;
    
    bx_c_dot = (controllers.phy_controller.p * (bx_c - R(1, 3)));
    by_c_dot = (controllers.theta_controller.p * (by_c - R(2, 3)));
    
    res = tf_mat * [bx_c_dot; by_c_dot];
    
    torques(1) = res(1);
    torques(2) = res(2);
    % torques(3) = ((controllers.psy_controller.d * (desired_state.thetadot(3) - state.thetadot(3))) + ...
    %            (controllers.psy_controller.p * (desired_state.theta(3) - state.theta(3)))) * Izz;
    torques(3) = 0;
        
end