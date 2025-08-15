%% Cart-Pole System Simulation without External Force

% Time parameters
t_start = 0;
t_end = 5;
dt = 0.01;
t = t_start:dt:t_end;

% Physical parameters (typical values)
M = 1.0;    % Mass of cart (kg)
m = 0.1;    % Mass of pendulum (kg)
l = 0.5;    % Length of pendulum (m)
g = 9.81;   % Gravitational acceleration (m/s^2)

% Initial conditions
% [x, x_dot, theta, theta_dot]
x0 = 0;         % Initial cart position (m)
x_dot0 = 0;     % Initial cart velocity (m/s)
theta0 = 0.2;   % Initial pendulum angle (rad) - small perturbation
theta_dot0 = 0; % Initial pendulum angular velocity (rad/s)

initial_conditions = [x0; x_dot0; theta0; theta_dot0];

% Solve the differential equation using ode45
[t_sol, y] = ode45(@(t, y) cartpole_dynamics(t, y, M, m, l, g), t, initial_conditions);

% Extract solutions
x = y(:, 1);           % Cart position
x_dot = y(:, 2);       % Cart velocity
theta = y(:, 3);       % Pendulum angle
theta_dot = y(:, 4);   % Pendulum angular velocity

% Convert theta to degrees for better visualization
theta_deg = theta * 180 / pi;

% Create plots
figure('Position', [100, 100, 1200, 800]);

% Plot 1: Cart position vs time
subplot(2, 2, 1);
plot(t_sol, x, 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Cart Position x (m)');
title('Cart Position vs Time');
grid on;

% Plot 2: Pendulum angle vs time (in degrees)
subplot(2, 2, 2);
plot(t_sol, theta_deg, 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Pendulum Angle θ (degrees)');
title('Pendulum Angle vs Time');
grid on;

% Plot 3: Phase portrait for cart (position vs velocity)
subplot(2, 2, 3);
plot(x, x_dot, 'b-', 'LineWidth', 1.5);
xlabel('Cart Position x (m)');
ylabel('Cart Velocity dx/dt (m/s)');
title('Cart Phase Portrait');
grid on;

% Plot 4: Phase portrait for pendulum (angle vs angular velocity)
subplot(2, 2, 4);
plot(theta, theta_dot, 'r-', 'LineWidth', 1.5);
xlabel('Pendulum Angle θ (rad)');
ylabel('Angular Velocity dθ/dt (rad/s)');
title('Pendulum Phase Portrait');
grid on;

% Add overall title
sgtitle('Cart-Pole System Simulation (F_{ext} = 0)', 'FontSize', 16, 'FontWeight', 'bold');

% Display final values
fprintf('Final cart position: %.4f m\n', x(end));
fprintf('Final pendulum angle: %.4f rad (%.2f degrees)\n', theta(end), theta_deg(end));

% Animation (optional - uncomment to run)
% animate_cartpole(t_sol, x, theta, l);

%% Function: Cart-pole dynamics
function dydt = cartpole_dynamics(t, y, M, m, l, g)
    % Extract states
    x = y(1);
    x_dot = y(2);
    theta = y(3);
    theta_dot = y(4);
    
    % External force (set to 0 as specified)
    F_ext = 0;
    
    % Precompute trigonometric functions
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    
    % Denominator for the equations
    denominator = M + m - m * cos_theta^2;
    
    % Cart acceleration (from derived equations of motion)
    x_ddot = (F_ext + m * l * theta_dot^2 * sin_theta - m * g * sin_theta * cos_theta) / denominator;
    
    % Pendulum angular acceleration (from derived equations of motion)
    theta_ddot = (-F_ext * cos_theta - m * l * theta_dot^2 * sin_theta * cos_theta + (M + m) * g * sin_theta) / (l * denominator);
    
    % Return derivatives
    dydt = [x_dot; x_ddot; theta_dot; theta_ddot];
end

%% Optional Animation Function
function animate_cartpole(t, x, theta, l)
    figure('Position', [200, 200, 800, 600]);
    
    for i = 1:10:length(t)  % Skip frames for faster animation
        clf;
        
        % Cart position
        cart_x = x(i);
        cart_y = 0;
        
        % Pendulum bob position
        bob_x = cart_x + l * sin(theta(i));
        bob_y = l * cos(theta(i));
        
        % Draw cart
        rectangle('Position', [cart_x-0.1, cart_y-0.05, 0.2, 0.1], 'FaceColor', 'blue');
        hold on;
        
        % Draw pendulum rod
        plot([cart_x, bob_x], [cart_y, bob_y], 'k-', 'LineWidth', 3);
        
        % Draw pendulum bob
        plot(bob_x, bob_y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');
        
        % Draw ground
        plot([-2, 2], [-0.1, -0.1], 'k-', 'LineWidth', 2);
        
        % Set axis properties
        axis equal;
        xlim([min(x)-0.5, max(x)+0.5]);
        ylim([-0.2, l+0.1]);
        xlabel('Position (m)');
        ylabel('Height (m)');
        title(sprintf('Cart-Pole Animation (t = %.2f s)', t(i)));
        grid on;
        
        pause(0.01);
    end
end