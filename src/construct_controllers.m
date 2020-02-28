function controllers = construct_controllers(thrust_controller_params, phy_controller_params, ...
                                            theta_controller_params, psy_controller_params, ...
                                            x_position_controller_params, y_position_controller_params)
    % Integrates all controllers in a controller struct
    % Each controller can be a 'PD' or 'PID' contorller

    thrust_controller = params_to_struct(thrust_controller_params);
    phy_controller = params_to_struct(phy_controller_params);
    theta_controller = params_to_struct(theta_controller_params);
    psy_controller = params_to_struct(psy_controller_params);
    x_position_controller = params_to_struct(x_position_controller_params);
    y_position_controller = params_to_struct(y_position_controller_params);

    controllers = struct('thrust_controller', thrust_controller, 'phy_controller', phy_controller, ...
    'theta_controller', theta_controller, 'psy_controller', psy_controller, 'x_pos_controller', x_position_controller, ...
    'y_pos_controller', y_position_controller);

end

function controller = params_to_struct(controller_params)
    if size(controller_params, 1) == 2
        controller = struct('p', controller_params(1), 'd', controller_params(2));
    elseif size(controller_params, 1) == 3
        controller = struct('p', controller_params(1), 'i', controller_params(2), ...
            'd', controller_params(3));
    end
end
