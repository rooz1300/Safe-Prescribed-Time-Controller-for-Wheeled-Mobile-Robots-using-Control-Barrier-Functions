
% =========================================================================
% PRESCRIBED-TIME CONTROL OF A WHEELED MOBILE ROBOT (WMR)
% WITH CONTROL BARRIER FUNCTION (CBF) SAFETY FILTER
% =========================================================================
%
% Description:
% This script simulates the trajectory tracking of a nonholonomic Wheeled
% Mobile Robot (WMR). The control strategy consists of two main parts:
% 1. A Prescribed-Time Controller (PTC) that ensures the robot's tracking
%    error converges to zero within a user-defined time 'tp'. For times
%    greater than 'tp', a proportional controller is used for stabilization.
% 2. A Control Barrier Function (CBF) implemented as a safety filter. This
%    filter modifies the control inputs by solving a real-time Quadratic
%    Program (QP) to guarantee the robot avoids a predefined circular
%    obstacle, thus ensuring safety.
%
% The simulation is based on the methods described in the paper: "Safe
% prescribed time controller for wheeled mobile robots by using control
% barrier functions as a safety filter".
%
% Author: [Original Author Name]
% Comments Added By: Gemini
% Date: [Date of Commenting]
%
% =========================================================================

clc; clear; close all;

% Obstacle parameters
global center radius

% Define the center coordinates of the circular obstacle.
center = [3.7; 11];
% Define the radius of the circular obstacle.
radius = 1;

% The 'alpha' parameter is part of the Exponential CBF (ECBF) constraint,
% influencing how conservatively the robot avoids the obstacle.

alpha_values = [2];
% Define the initial position [x, y, phi] for the robot systems.
intial_pos = [-1, -1, 0];

% Define the initial conditions for the system's states.
% This vector includes reference trajectory states, actual system states,
% gamma parameters, and safe system states.
initial_conditions_r = [-0.5, 0.2, 0, intial_pos, 30, 10, intial_pos]';

% Define the time span for the simulation, from 0 to 8 seconds with 100 points.
tspan = linspace(0, 8, 100);

% Initialize a structure to store the simulation results for each alpha value.
results = struct('t', [], 'y', [], 'v_in_vals', [], 'omega_in_vals', [], 'x_e_vals', [], 'y_e_vals', [], 'phi_e_vals', []);

% Loop through each alpha value to run the simulation.
for k = 1:length(alpha_values)
    % Set the current alpha value for the simulation.
    alpha = alpha_values(k);

    % Solving the ODE
    % Use the custom Euler method to solve the ordinary differential equations
    % defined in 'main_system'. The 'v_refence' and 'omega_refence' functions
    % provide reference velocities.
    [t, y] = euler(@(t, y) main_system(t, y, v_refence(t), omega_refence(t), alpha), tspan, initial_conditions_r);

    % Get the number of time steps.
    n = length(t);

    % Initialize arrays to store control inputs and error states for plotting.
    v_in_vals = zeros(1, n);
    omega_in_vals = zeros(1, n);
    x_e_vals = zeros(1, n);
    y_e_vals = zeros(1, n);
    phi_e_vals = zeros(1, n);

    % Calculate control inputs and error states at each time step.
    for i = 1:n
        [v_in_vals(1, i), omega_in_vals(1, i), x_e_vals(1, i), y_e_vals(1, i), phi_e_vals(1, i)] = ...
            calculate_control_inputs(y(i, 1), y(i, 2), y(i, 3), y(i, 4), y(i, 5), y(i, 6), y(i, 7), y(i, 8), v_refence(t(i)), omega_refence(t(i)), t(i));
    end

    % Save results into the structure array for the current alpha value.
    results(k).t = t; % Time vector
    results(k).y = y; % State trajectories (reference, actual, safe)
    results(k).v_in_vals = v_in_vals; % Linear velocity control inputs
    results(k).omega_in_vals = omega_in_vals; % Angular velocity control inputs
    results(k).x_e_vals = x_e_vals; % X-axis error
    results(k).y_e_vals = y_e_vals; % Y-axis error
    results(k).phi_e_vals = phi_e_vals; % Orientation error

    disp('RUN successfully.');
end


%% Plotting the results for each variable across all alpha values

% Create a new figure for plotting the states over time.
figure('Position', [100, 100, 1000, 500]); % Adjust width and height as needed

% Subplot for X-position vs. Time.
subplot(3, 1, 1);
% Plot the unsafe trajectory's x-position.
plot(results(1).t, results(1).y(:, 4), 'DisplayName', 'unsafe ', 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 3);
hold on;
% Plot the reference trajectory's x-position.
plot(results(1).t, results(1).y(:, 1), 'DisplayName', 'reference ', 'Color', 'red', 'LineStyle', '--', 'LineWidth', 3);

% Define line styles for different alpha values for visual distinction.
line_styles = {'-', '--', ':', '-.'}; % Different line styles

% Plot the safe trajectory's x-position for all alpha values.
for k = 1:length(alpha_values)
    style = line_styles{mod(k - 1, length(line_styles)) + 1}; % Cycle through line styles
    plot(results(k).t, results(k).y(:, 9), 'Color', "black", 'LineStyle', style, 'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    hold on;
end
title('x vs Time for all $\alpha$ values', 'Interpreter', 'latex');
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('x [m]', 'Interpreter', 'latex');
legend('Location', 'eastoutside', 'Interpreter', 'latex');
grid minor;
hold off;

% Plotting y vs Time for all alpha values.
subplot(3, 1, 2);
% Plot the unsafe trajectory's y-position.
plot(results(1).t, results(1).y(:, 5), 'DisplayName', 'unsafe trajectory', 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 3);
hold on;
% Plot the reference trajectory's y-position.
plot(results(1).t, results(1).y(:, 2), 'DisplayName', 'reference trajectory', 'Color', 'red', 'LineStyle', '--', 'LineWidth', 3);
hold on;
% Plot the safe trajectory's y-position for all alpha values.
for k = 1:length(alpha_values)
    style = line_styles{mod(k - 1, length(line_styles)) + 1};
    plot(results(k).t, results(k).y(:, 10), 'Color', "black", 'LineStyle', style, 'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    hold on;
end
title('y vs Time for all $\alpha$ values', 'Interpreter', 'latex');
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');
legend('Location', 'eastoutside', 'Interpreter', 'latex');
grid minor;
hold off;

% Plotting phi (orientation) vs Time for all alpha values.
subplot(3, 1, 3);
% Plot the unsafe trajectory's orientation.
plot(results(1).t, results(1).y(:, 6), 'DisplayName', 'unsafe trajectory', 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 3);
hold on;
% Plot the reference trajectory's orientation.
plot(results(1).t, results(1).y(:, 3), 'DisplayName', 'reference trajectory', 'Color', 'red', 'LineStyle', '--', 'LineWidth', 3);
hold on;
% Plot the safe trajectory's orientation for all alpha values.
for k = 1:length(alpha_values)
    style = line_styles{mod(k - 1, length(line_styles)) + 1};
    plot(results(k).t, results(k).y(:, 11), 'Color', "black", 'LineStyle', style, 'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    hold on;
end
title('$\phi$ vs Time for all $\alpha$ values', 'Interpreter', 'latex');
xlabel('Time [sec]', 'Interpreter', 'latex');
ylabel('$\phi$ [rad]', 'Interpreter', 'latex');
legend('Location', 'eastoutside', 'Interpreter', 'latex');
grid minor;
hold off;

% Save the figure to a PNG file with high resolution.
print('figure_9_states_for_all_lapha_values', '-dpng', '-r600');


%% Plot reference and actual paths

% Create a new figure for plotting the 2D paths.
figure('Position', [0, 0, 800, 700]); % Adjust width and height as needed

% Define line styles for different alpha values.
line_styles = {'-', '--', ':', '-.'};

% Plot the safe trajectories (x vs y) for all alpha values.
hold on;
for k = 1:length(alpha_values)
    style = line_styles{mod(k - 1, length(line_styles)) + 1};
    plot(results(k).y(:, 9), results(k).y(:, 10), 'Color', "black", 'LineStyle', style, 'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    hold on;
end

% Plot the unsafe trajectory (x vs y).
plot(results(1).y(:, 4), results(1).y(:, 5), 'DisplayName', 'unsafe trajectory', 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 3);
hold on;

% Plot the reference trajectory (x vs y).
plot(results(1).y(:, 1), results(1).y(:, 2), 'DisplayName', 'reference trajectory', 'Color', 'red', 'LineStyle', '--', 'LineWidth', 3);

title('Reference Path vs Actual Path for all $\alpha$ values', 'Interpreter', 'latex');
xlabel('x [m]', 'Interpreter', 'latex');
ylabel('y [m]', 'Interpreter', 'latex');
legend('Location', 'eastoutside', 'Interpreter', 'latex');
grid minor;
hold on;

% Obstacle plot
% Generate points for drawing a circle representing the obstacle.
theta = linspace(0, 2 * pi, 100);
x_c = center(1) + radius * cos(theta);
y_c = center(2) + radius * sin(theta);

% Plot the circle boundary.
plot(x_c, y_c, 'Color', [0, 1, 1], 'LineWidth', 1, 'DisplayName', 'Obstacle'); % RGB color for green

% Define the grid density for the obstacle's internal lines.
numLines = 20;

% Generate and plot horizontal lines within the circle to fill it.
for y = linspace(center(2) - radius, center(2) + radius, numLines)
    if abs(y - center(2)) <= radius
        x_intersect = sqrt(radius^2 - (y - center(2))^2);
        plot([center(1) - x_intersect, center(1) + x_intersect], [y, y], 'Color', [0.5, 0.5, 0.5], 'HandleVisibility', 'off');
    end
end

% Generate and plot vertical lines within the circle to fill it.
for x = linspace(center(1) - radius, center(1) + radius, numLines)
    if abs(x - center(1)) <= radius
        y_intersect = sqrt(radius^2 - (x - center(1))^2);
        plot([x, x], [center(2) - y_intersect, center(2) + y_intersect], 'Color', [0.5, 0.5, 0.5], 'HandleVisibility', 'off');
    end
end

movegui center; % Center the figure on the screen.
xlim([-9 9]); % Set the x-axis limits.
axis equal; % Set equal scaling for x and y axes.
grid on; % Turn on the grid.
hold off; % Release the hold on the plot.

% Save the figure to a PNG file with high resolution.
print('figure_11_Reference_Pathvs Actual Path for all alphs values', '-dpng', '-r600');


%% Control Inputs Plotting

% Define line styles for different alpha values.
line_styles = {'-', '--', ':', '-.'};

% Create a new figure for plotting the control inputs.
figure('Position', [100, 100, 1200, 450]);

% Loop through each alpha value to plot its corresponding control inputs.
for k = 1:length(alpha_values)
    style = line_styles{mod(k - 1, length(line_styles)) + 1};

    % Extract data for current alpha value.
    x = results(k).y(:, 9);
    phi = results(k).y(:, 11);
    t = results(k).t;

    % Calculate control inputs (omega_in, v_in) from the safe trajectory states.
    [omega_in, v_in] = controlInputs_cal(results(k).y(:, 9), results(k).t, results(k).y(:, 11));

    % Store calculated control inputs in the results structure.
    results(k).omega_in = omega_in;
    results(k).v_in = v_in;

    % Plot linear velocity (v_in) in the first subplot.
    subplot(2, 1, 1);
    plot(t(1:end - 1), v_in(1:end - 1), 'Color', 'black', 'LineStyle', style, ...
        'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    title('$v$ vs Time for all $\alpha$ values', 'Interpreter', 'latex');
    xlabel('Time [sec]', 'Interpreter', 'latex');
    ylabel('$v[m/sec]$', 'Interpreter', 'latex');
    grid minor;
    hold on; % Hold on to plot multiple lines on the same subplot.

    % Plot angular velocity (omega_in) in the second subplot.
    subplot(2, 1, 2);
    plot(t(1:end - 1), omega_in(1:end - 1), 'Color', 'black', 'LineStyle', style, ...
        'DisplayName', ['$\alpha$ = ', num2str(alpha_values(k))], 'LineWidth', 1);
    hold on;
    ylim([-5, 5]); % Set y-axis limits for omega.
    xlim([1, 7.5]); % Set x-axis limits for omega.
    title('$\omega$ vs Time for all $\alpha$ values', 'Interpreter', 'latex');
    xlabel('Time [sec]', 'Interpreter', 'latex');
    ylabel('$\omega[rad/sec]$', 'Interpreter', 'latex');
    grid minor;
end

% Add legends to the subplots.
subplot(2, 1, 1);
legend('show', 'Interpreter', 'latex', 'Location', 'eastoutside');

subplot(2, 1, 2);
legend('show', 'Interpreter', 'latex', 'Location', 'eastoutside');



% Save the workspace, including all variables and results.
% save(['results\WMR_control_', datestr(now, 'yyyy_mm_dd_HH_MM_SS'), '.mat']);
% disp('Workspace saved successfully.');




%% ========================================================================
%                     FUNCTION DEFINITIONS
% =========================================================================

function dydt = main_system(t, states, v_r, omega_r, alpha)
% main_system defines the differential equations for the robot's dynamics
% and the control barrier function (CBF) optimization problem.
%
% Inputs:
%   t: Current time.
%   states: Column vector of current state variables:
%           [x_r; y_r; phi_r; x; y; phi; gamma_1; gamma_2; x_safe; y_safe; phi_safe]
%           where (x_r, y_r, phi_r) are reference states,
%           (x, y, phi) are unsafe system states,
%           (gamma_1, gamma_2) are adaptive controller parameters  (currently unused in dynamics),
%           and (x_safe, y_safe, phi_safe) are safe system states.
%   v_r: Reference linear velocity.
%   omega_r: Reference angular velocity.
%   alpha: Controller parameter used in the CBF constraint.
%
% Output:
%   dydt: Column vector of the derivatives of the state variables.

% Obstacle parameters are global.
global center radius;

% Unpack the state variables for better readability.
x_r = states(1); % Reference x-position
y_r = states(2); % Reference y-position
phi_r = states(3); % Reference orientation

x = states(4); % Unsafe system x-position
y = states(5); % Unsafe system y-position
phi = states(6); % Unsafe system orientation

gamma_1 = states(7); % Controller parameter 1 (currently set to 0 in dynamics)
gamma_2 = states(8); % Controller parameter 2 (currently set to 0 in dynamics)

x_safe = states(9); % Safe system x-position
y_safe = states(10); % Safe system y-position
phi_safe = states(11); % Safe system orientation

% Calculate errors between reference and unsafe actual system states
% in the robot's body frame.
temp = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1] * [x_r - x; y_r - y; phi_r - phi];
x_e = temp(1); % Error in x-direction
y_e = temp(2); % Error in y-direction
phi_e = temp(3); % Error in orientation

% Calculate errors between reference and safe actual system states
% in the safe robot's body frame.
temp2 = [cos(phi_safe), sin(phi_safe), 0; -sin(phi_safe), cos(phi_safe), 0; 0, 0, 1] * [x_r - x_safe; y_r - y_safe; phi_r - phi_safe];
x_e_safe = temp2(1); % Error in x-direction for safe system
y_e_safe = temp2(2); % Error in y-direction for safe system
phi_e_safe = temp2(3); % Error in orientation for safe system

%% Control Input Calculation

% Calculate control inputs for the unsafe system using the 'controller' function.
[v_in, omega_in] = controller(gamma_1, gamma_2, v_r, omega_r, x_e, y_e, phi_e, t);
u_unsafe = [v_in, omega_in]; % Unsafe control input vector

% Calculate control inputs for the safe system (before CBF modification).
[v_in_safe, omega_in_safe] = controller(gamma_1, gamma_2, v_r, omega_r, x_e_safe, y_e_safe, phi_e_safe, t);
u_safe = [v_in_safe; omega_in_safe]; % Desired control input for safe system

% Clear YALMIP variables to avoid conflicts in subsequent QP solves.
yalmip('clear');
ucbf = sdpvar(2, 1); % Define optimization variable for CBF-modified control input [v; omega]
slack = sdpvar; % Define slack variable (currently unused in the objective/constraints but declared)

%% CBF Constraint Quadratic Program (QP)

% Define the Control Barrier Function (CBF), h(x) = (x - x_c)^2 + (y - y_c)^2 - R^2.
% hsafe > 0 implies the robot is outside the obstacle.
hsafe = (x_safe - center(1))^2 + (y_safe - center(2))^2 - radius^2;

% Calculate the first derivative of the CBF (h_dot).
% h_dot = grad_h * f(x, u)
% For a unicycle model: x_dot = v*cos(phi), y_dot = v*sin(phi)
% h_dot = 2*(x_safe - center(1))*v*cos(phi_safe) + 2*(y_safe - center(2))*v*sin(phi_safe)
hdot = 2 * (x_safe - center(1)) * ucbf(1) * cos(phi_safe) + 2 * (y_safe - center(2)) * ucbf(1) * sin(phi_safe);

% Calculate the second derivative of the CBF (h_double_dot).
% This is a simplified representation assuming linearity for QP, or specific structure of CBF.
% More rigorously, this would involve derivatives with respect to omega as well.
h2dot = [2 * ucbf(1), 2 * ucbf(1) * ((y_safe - center(2)) * cos(phi_safe) - (x_safe - center(1)) * sin(phi_safe))] * ucbf;

% Define the CBF constraint: h_double_dot + 2*alpha*h_dot + alpha^2*h >= 0.
% This ensures that the system states remain safe.
const = h2dot + 2 * alpha * hdot + alpha^2 * hsafe >= 0;

% Objective function for the Quadratic Program.
% Minimize the difference between the CBF-modified control input (ucbf)
% and the desired (unmodified) safe control input (u_safe).
obj = 0.5 * norm(ucbf - u_safe)^2;

% YALMIP solver settings. 'verbose' set to 0 to suppress solver output.
ops = sdpsettings('solver', '', 'verbose', 0);

% Solve the Quadratic Program to find the optimal CBF-modified control input.
optimize(const, obj, ops);
ucbf_opt = value(ucbf); % Extract the optimal control input.

%% System Dynamics

% Dynamics of the reference trajectory (constant linear and angular velocity).
dx_rdt = v_r * cos(phi_r);
dy_rdt = v_r * sin(phi_r);
dphi_r = omega_r;

% Dynamics of the actual (unsafe) system, driven by the calculated unsafe control inputs.
dx = u_unsafe(1) * cos(phi);
dy = u_unsafe(1) * sin(phi);
dphi = u_unsafe(2);

% Derivatives of gamma parameters (currently set to zero, implying constant values).
dgamma_1 = 0;
dgamma_2 = 0;

% Dynamics of the safe system, driven by the CBF-modified control inputs.
dx_safe = ucbf_opt(1) * cos(phi_safe);
dy_safe = ucbf_opt(1) * sin(phi_safe);
dphi_safe = ucbf_opt(2);

% Concatenate all derivatives into a single column vector.
dydt = [dx_rdt; dy_rdt; dphi_r; dx; dy; dphi; dgamma_1; dgamma_2; dx_safe; dy_safe; dphi_safe];

end

function out = v_refence(~)
% v_refence provides a constant reference linear velocity.
% Input: ~ (time, not used, as velocity is constant)
% Output: out (constant linear velocity)
out = 5; % Meters per second
end

function out = omega_refence(~)
% omega_refence provides a constant reference angular velocity.
% Input: ~ (time, not used, as angular velocity is constant)
% Output: out (constant angular velocity)
out = 0.8; % Radians per second
end

function [v_in, omega_in] = controller(gamma_1, gamma_2, v_r, omega_r, x_e, y_e, phi_e, t)
% controller calculates the control inputs (linear and angular velocity)
% based on tracking errors and a time-dependent switching logic.
%
% Inputs:
%   gamma_1, gamma_2: Controller gain parameters (redefined locally, so input values are not used).
%   v_r: Reference linear velocity.
%   omega_r: Reference angular velocity.
%   x_e, y_e, phi_e: Tracking errors in x, y, and orientation, respectively.
%   t: Current time.
%
% Outputs:
%   v_in: Calculated linear velocity control input.
%   omega_in: Calculated angular velocity control input.

% Define a time threshold for controller switching.
t_p = 1;

% Redefine gamma values locally, overriding the input gamma_1, gamma_2.
gamma_1 = 1.1;
gamma_2 = 1.1;

% Fault factors (currently set to 1, implying no fault/scaling).
F_constant_omega = 1;
F_constant_v = 1;

% Apply a time-dependent control law.
% If time is less than t_p, use a specific control law (possibly for initial stabilization).
if t <= t_p - 0.0000001 % Small epsilon to handle floating point comparisons
    % Control laws designed to reduce errors, potentially based on a non-linear approach.
    omega_in_t = F_constant_omega * ((v_r * sin(phi_e) - (gamma_1 * y_e) / (t - t_p)) / x_e);
    v_in_t = F_constant_v * (v_r * cos(phi_e) + (y_e * (v_r * sin(phi_e) - (gamma_1 * y_e) / (t - t_p))) / x_e - (gamma_2 * x_e) / (t - t_p));
else
    % After t_p, switch to a proportional-like controller with different gains.
    k1 = 5; % Control gain for linear velocity.
    k2 = 5; % Control gain for angular velocity.

    v_in_t = F_constant_v * k1 * x_e;
    omega_in_t = F_constant_omega * k2 * atan2(y_e, x_e); % atan2 is used for robust angle calculation.
end

% Apply saturation limits to the control inputs to keep them within physical bounds.
v_in = min(10, max(-10, v_in_t)); % Linear velocity limited to [-10, 10] m/s.
omega_in = min(3, max(-3, omega_in_t)); % Angular velocity limited to [-3, 3] rad/s.

end

function [v_in, omega_in, x_e, y_e, phi_e] = calculate_control_inputs(x_r, y_r, phi_r, x, y, phi, gamma_1, gamma_2, v_r, omega_r, t)
% calculate_control_inputs calculates the errors and then calls the 'controller'
% function to get the actual control inputs for plotting and analysis.
%
% Inputs:
%   x_r, y_r, phi_r: Reference states (position and orientation).
%   x, y, phi: Actual system states.
%   gamma_1, gamma_2: Controller parameters.
%   v_r, omega_r: Reference linear and angular velocities.
%   t: Current time.
%
% Outputs:
%   v_in: Calculated linear velocity control input.
%   omega_in: Calculated angular velocity control input.
%   x_e, y_e, phi_e: Calculated tracking errors.

% Calculate errors between reference and actual system states
% in the actual robot's body frame.
temp = [cos(phi), sin(phi), 0; -sin(phi), cos(phi), 0; 0, 0, 1] * [x_r - x; y_r - y; phi_r - phi];
x_e = temp(1); % Error in x-direction
y_e = temp(2); % Error in y-direction
phi_e = temp(3); % Error in orientation

% Get control inputs using the 'controller' function.
[v_in, omega_in] = controller(gamma_1, gamma_2, v_r, omega_r, x_e, y_e, phi_e, t);

end

function [omega_in, v_in] = controlInputs_cal(x, t, phi)
% controlInputs_cal calculates linear and angular velocities
% by numerically differentiating the position and orientation data.
% This function seems to be used for back-calculating inputs from states,
% rather than for forward control.
%
% Inputs:
%   x: X-position trajectory.
%   t: Time vector.
%   phi: Orientation trajectory.
%
% Outputs:
%   omega_in: Calculated angular velocity.
%   v_in: Calculated linear velocity.

dx_dt = gradient(x', t'); % Numerical derivative of x with respect to time.
omega_in = gradient(phi', t'); % Numerical derivative of phi (angular velocity).
v_in = dx_dt .* (1 ./ cos(phi')); % Linear velocity derived from dx/dt and orientation.

end

function [t, y] = euler(odefun, tspan, y0)
% euler solves ordinary differential equations (ODEs) using the Euler method.
% This is a custom implementation of a numerical ODE solver.
%
% Inputs:
%   odefun: Function handle of the form @(t, y) f(t, y, ...), which defines
%           the right-hand side of the ODEs.
%   tspan: Either a two-element vector [t0 tf] specifying the start and end
%          times, or a vector of specific time points at which to solve.
%   y0: Initial conditions for the state variables (column vector).
%
% Outputs:
%   t: Column vector of time points at which the solution is computed.
%   y: Matrix where each row corresponds to the solution (state vector)
%      at the corresponding time point in 't'.

% Default step size for fixed step Euler method. Can be adjusted for accuracy.
h = 0.01;

% Handle tspan input: either [t0 tf] or a predefined vector of time points.
if length(tspan) == 2
    t0 = tspan(1);
    tf = tspan(2);
    t = (t0:h:tf)'; % Generate time vector with fixed step size.
    if t(end) < tf
        t = [t; tf]; % Ensure the final time point is included.
    end
else
    t = tspan(:); % Use the provided time points directly.
    h = diff(t); % Calculate variable step sizes if tspan is a vector of points.
end

% Initialize the output matrix for the solution.
n = length(t); % Number of time points.
y = zeros(n, length(y0)); % Pre-allocate matrix for state trajectories.
y(1, :) = y0(:)'; % Set the initial conditions (first row).

% Euler method loop: iterate through time steps to compute the solution.
for i = 1:n - 1
    % Current time and state.
    t_current = t(i);
    y_current = y(i, :)'; % Transpose to a column vector for odefun.

    % Evaluate the ODE function to get the derivatives at the current state.
    dydt = odefun(t_current, y_current);

    % Euler step: y(t+h) = y(t) + h * f(t, y).
    % Use fixed or variable step size depending on how tspan was defined.
    if length(tspan) == 2
        y(i + 1, :) = y(i, :) + h * dydt';
    else
        y(i + 1, :) = y(i, :) + h(i) * dydt';
    end
end
end