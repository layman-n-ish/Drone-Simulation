# Drone-Simulation

Simulates the dynamics of a quadcopter and provides controllers (PID) to fly the quadcopter given initial conditions. Designed to run tests to study the effects of controller parameters or tests to study the response of the quadcopter with different intial conditions and desired goals.  

This is an extension of @[gibiansky's](https://github.com/gibiansky/experiments/tree/master/quadcopter) codebase. The dynamics were studied from Teppo Luukkonen's technical paper on '[Modelling and control of quadcopter](https://sal.aalto.fi/publications/pdf-files/eluu11_public.pdf)'.

### Code Structure

`src/construct_controllers.m`: Amalgamates controllers in a `struct` to control the drone in 6-DoF. Each controller can be a stand-alone PID or PD controller. 

`src/rotation.m`: Computes the rotation matrix to transform from the body frame to inertial frame, given the attitude (angular displacements) of the drone.

`src/resolve_control_signals.m`: This is the crucial function that computes the control signals for each of the motors to reach the desired immediate goal given the current state of the quadcopter. [A lot of assumptions are made here; the global co-ordinates and attitude of the drone is tapped in directly with no noise] 

`src/simulate.m`: Simulates the flying of the drone by computing the control signals for each the motors and advancing the drone's state by the dynamics its confined to. Also defines the physical constants and the mathematical model of the quadcopter. 

`src/visualize.m`: Visualizes the complete run which is saved by the `src/simulate.m` script.

`src/plot_data.m`: Plots both linear and angular statistics of the drone during the run along with the control signals supplied to each of the four motors. 

### Running Simulations

Once you initialize the controller with their parameters,
```
controllers = construct_controllers([1.5; 2.5], [6; 1.75], [6; 1.75], [6; 1.75]);
```

and set initial conditions and desired goals,
```
init_state = struct('x', 0, 'xdot', 0, 'theta', [45; 45; 45], 'thetadot', [0; 0; 0]);
desired_state = struct('z', 10, 'zdot', 0, 'theta', [0; 0; 0], 'thetadot', [0; 0; 0]);
```

you can run the simulation by specifying the init. time (`tstart`), the run's end time (`tend`) and duration of the time step (`dt`):
```
simulate(controllers, desired_state, init_state, tstart, tend, dt);
```

To visualize or plot graphs of the simulated run of the quadcopter, execute:
```
visualize(simulate(controllers, desired_state, init_state, tstart, tend, dt));
plot_data(simulate(controllers, desired_state, init_state, tstart, tend, dt));
```

---

An example to hover the drone at an height (10m) starting in a disoriented manner from the ground up has been implemented in `tests/hover_stabilize.m`. To execute the test run from the repo's home directory:
```
run(tests/hover_stabilize.m)
```
Output of the above simulation:
![Drone hovering demo](https://github.com/layman-n-ish/Drone-Simulation/blob/master/demos/hover_stabilise.gif)
![Drone hovering plots](https://github.com/layman-n-ish/Drone-Simulation/blob/master/imgs/hover_stabilize.jpg)

#### To Do:
- Show demos and walk through the inferences of the various tests simulated
- Finish up `position-controller` branch and merge with the `master`
- Add support for 'Integral' controller (for PID)
- Implement Model Predictive Control (MPC) based controller
