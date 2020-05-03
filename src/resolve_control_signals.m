% computes the control signals (of the 4 motors) to reach the desired goals
function [control_signals, state] = resolve_control_signals(controllers, state, desired_state, theta, thetadot, z, zdot)
    thrust = compute_thrust(controllers.thrust_controller, state, desired_state, theta, z, zdot);
    torques = compute_torques(controllers, state, desired_state, theta, thetadot);
    
    control_signals = zeros(4, 1);
    control_signals(1) = (thrust/(4*state.k)) - (torques(2)/(2*state.k*state.L)) - (torques(3)/(4*state.b));
    control_signals(2) = (thrust/(4*state.k)) - (torques(1)/(2*state.k*state.L)) + (torques(3)/(4*state.b));
    control_signals(3) = (thrust/(4*state.k)) + (torques(2)/(2*state.k*state.L)) - (torques(3)/(4*state.b));
    control_signals(4) = (thrust/(4*state.k)) + (torques(1)/(2*state.k*state.L)) + (torques(3)/(4*state.b));
end

function thrust = compute_thrust(thrust_controller, state, desired_state, theta, z, zdot)
    err_d = thrust_controller.d * (desired_state.zdot - zdot);
    err_p = thrust_controller.p * (desired_state.z - z);
    
    thrust = ((state.g + err_d + err_p) * state.m) / (cos(theta(1)) * cos(theta(2)));
end

function torques = compute_torques(controllers, state, desired_state, theta, thetadot)
    torques = zeros(3, 1);
    
    Ixx = state.I(1, 1);
    Iyy = state.I(2, 2);
    Izz = state.I(3, 3);
    
    torques(1) = ((controllers.phy_controller.d * (desired_state.thetadot(1) - thetadot(1))) + ...
                (controllers.phy_controller.p * (desired_state.theta(1) - theta(1)))) * Ixx;
            
    torques(2) = ((controllers.theta_controller.d * (desired_state.thetadot(2) - thetadot(2))) + ...
                (controllers.theta_controller.p * (desired_state.theta(2) - theta(2)))) * Iyy;
    
    torques(3) = ((controllers.psy_controller.d * (desired_state.thetadot(3) - thetadot(3))) + ...
                (controllers.psy_controller.p * (desired_state.theta(3) - theta(3)))) * Izz;
        
end