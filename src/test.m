kp = 6; kd = 1.75;

thrust_c_params = [1.5; 2.5]; % altitude control
phy_c_params = [kp; kd]; % attitude control
theta_c_params = [kp; kd]; % attitude control
psy_c_params = [kp; kd]; % attitude control
x_pos_params = [2; 1]; % position control
y_pos_params = [2; 1]; % position control
controllers = construct_controllers(thrust_c_params, ...
                                    phy_c_params, theta_c_params, psy_c_params, ...
                                    x_pos_params, y_pos_params);

x = [0; 0; 0];
xdot = [0; 0; 0];
theta = deg2rad([10; 10; 10]);
thetadot = deg2rad([0; 0; 5]);
init_state = struct('x', x, 'xdot', xdot, 'theta', theta, 'thetadot', thetadot);

desired_x = [10; 10; 10];
desired_xdot = [0; 0; 0];
desired_theta = deg2rad([0; 0; 20]);
desired_thetadot = deg2rad([0; 0; 0]);
desired_state = struct('x', desired_x, 'xdot', desired_xdot, 'theta', desired_theta, 'thetadot', desired_thetadot);

plot_data(simulate(controllers, desired_state, init_state, 0, 10, 0.001));