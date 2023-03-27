% ATTITUDE_SIMULATION simulates the attitude control of a spacecraft.
%
% This script demonstrates the use of a control law for the attitude
% control of a spacecraft. It defines the system parameters, solves the
% attitude dynamics, and plots the results for both the true and desired
% attitudes, as well as the attitude error over time.
%
% The script consists of the following main functions:
%   - init_params: Initializes the system parameters.
%   - solve_dynamics: Solves the attitude dynamics using the ode45 solver.
%   - attitude_dynamics: Calculates the attitude dynamics based on the
%     control law and system parameters.
%   - reference_dynamics: Defines the desired reference dynamics for the
%     spacecraft's attitude.
%   - tilde_dynamics: Calculates the tilde dynamics for error computation.
%   - control_law: Implements the control law.
%   - plot_and_save_results: Plots and saves the simulation results.
%
% Usage:
%   attitude_simulation()
%
% See also: ode45
%
% Author: Alexander Little
% Affiliation: Toronto Metropolitan University
% Date: 2023-03-26
% Version: 1.0

function attitude_simulation()
    params = init_params();
    [t, y] = solve_dynamics(params);
    plot_and_save_results(t, y);
end

% Initialize system parameters
function params = init_params()
    params = struct('hub_inertia', 1, 'appendage_length', 1, ... 
        'hub_length', 1, 'mass_per_unit_length', 1, 'derivative_gain', ... 
        1, 'convergence_rate', 1);
    params.total_inertia = params.hub_inertia + (2 * ... 
        params.appendage_length * params.mass_per_unit_length * (3 * ... 
        params.hub_length ^ 2 + 3 * params.hub_length * ... 
        params.appendage_length + params.appendage_length ^ 2)) / 3;
end

% Solve the attitude dynamics
function [t, y] = solve_dynamics(params)
    y0 = [0; 0];
    tspan = [0 20];
    options = odeset('RelTol', 1e-12, 'AbsTol', 1e-15);
    [t, y] = ode45(@(t, y) attitude_dynamics(t, y, params), tspan, y0, ... 
        options);
end

% Calculate the attitude dynamics
function dydt = attitude_dynamics(t, y, params)
    attitude = y(1);
    angular_velocity = y(2);

    [desired_attitude, desired_angular_velocity, ... 
        desired_angular_acceleration] = reference_dynamics(t);

    [attitude_error, angular_velocity_error] = ... 
        tilde_dynamics(attitude, angular_velocity, desired_attitude, ... 
        desired_angular_velocity);

    control_input = control_law(attitude_error, ... 
        angular_velocity_error, desired_angular_acceleration, params);

    angular_acceleration = control_input / params.total_inertia;
    
    dydt = [angular_velocity; angular_acceleration];
end

% Calculate the reference dynamics
function [desired_attitude, desired_angular_velocity, ... 
    desired_angular_acceleration] = reference_dynamics(t)
    desired_attitude = sin(t);
    desired_angular_velocity = cos(t);
    desired_angular_acceleration = -sin(t);
end

% Calculate the tilde dynamics
function [attitude_error, angular_velocity_error] = ... 
    tilde_dynamics(attitude, angular_velocity, desired_attitude, ... 
    desired_angular_velocity)
    attitude_error = attitude - desired_attitude;
    angular_velocity_error = angular_velocity - desired_angular_velocity;
end

% Calculate the control law
function control_input = control_law(attitude_error, ... 
    angular_velocity_error, desired_angular_acceleration, params)
    control_input = params.total_inertia * ... 
        (desired_angular_acceleration - params.convergence_rate * ... 
        angular_velocity_error) - params.derivative_gain * ... 
        (angular_velocity_error + params.convergence_rate * ... 
        attitude_error);
end

% Plot and save the simulation results
function plot_and_save_results(t, y)
    figure;
    subplot(2, 1, 1);
    plot(t, y(:, 1));
    hold on;
    plot(t, sin(t), 'r--');
    xlabel('Time (s)');
    ylabel('Attitude (rad)');
    title('Time History of True Attitude \theta(t)');
    legend('True Attitude', 'Desired Attitude');

    subplot(2, 1, 2);
    attitude_error = y(:, 1) - sin(t);
    plot(t, attitude_error);
    xlabel('Time (s)');
    ylabel('Attitude Error');
    title('Time History of Attitude Error');

    saveas(gcf, 'attitude_simulation_results', 'png');
end
