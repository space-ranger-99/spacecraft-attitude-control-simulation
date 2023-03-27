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
    theta = y(1);
    theta_dot = y(2);

    [theta_d, theta_d_dot, theta_d_ddot] = reference_dynamics(t);
    [theta_tilde, theta_tilde_dot] = tilde_dynamics(theta, theta_dot, ...
        theta_d, theta_d_dot);
    u = control_law(theta_tilde, theta_tilde_dot, theta_d_ddot, params);

    theta_ddot = u / params.total_inertia;
    dydt = [theta_dot; theta_ddot];
end

% Calculate the reference dynamics
function [theta_d, theta_d_dot, theta_d_ddot] = reference_dynamics(t)
    theta_d = sin(t);
    theta_d_dot = cos(t);
    theta_d_ddot = -sin(t);
end

% Calculate the tilde dynamics
function [theta_tilde, theta_tilde_dot] = tilde_dynamics(theta, ...
    theta_dot, theta_d, theta_d_dot)
    theta_tilde = theta - theta_d;
    theta_tilde_dot = theta_dot - theta_d_dot;
end

% Calculate the control law
function u = control_law(theta_tilde, theta_tilde_dot, theta_d_ddot, ...
    params)
    u = params.total_inertia * (theta_d_ddot - params.convergence_rate ...
        * theta_tilde_dot) - params.derivative_gain * (theta_tilde_dot ...
        + params.convergence_rate * theta_tilde);
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
    theta_error = y(:, 1) - sin(t);
    plot(t, theta_error);
    xlabel('Time (s)');
    ylabel('Attitude Error');
    title('Time History of Attitude Error');

    saveas(gcf, 'attitude_simulation_results', 'png');
end
