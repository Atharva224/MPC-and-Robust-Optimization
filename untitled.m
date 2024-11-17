% Define system parameters
T = 1;  % Time step in seconds
N = 10; % Prediction horizon
v_desired = 10; % Desired velocity (in m/s)
v0 = 0; % Initial velocity (in m/s)
umax = 1; % Max acceleration (in m/s²)
umin = -1; % Min acceleration (in m/s²)
disturbance_bounds = [-0.2, 0.2]; % Uncertainty in disturbances (in m/s)

% Setup optimization variables
u = sdpvar(N, 1); % Control inputs (acceleration) over the horizon
v = sdpvar(N+1, 1); % Velocities over the horizon
d = sdpvar(N, 1); % Disturbances (uncertain) over the horizon

% Define the system dynamics
constraints = [];
objective = 0;
v(1) = v0; % Initial condition

for k = 1:N
    % System dynamics with disturbance and time step
    v(k+1) = v(k) + T * u(k) + d(k);

    % Robust constraints on disturbance
    constraints = [constraints, disturbance_bounds(1) <= d(k) <= disturbance_bounds(2)];
    
    % Control input constraints (acceleration)
    constraints = [constraints, umin <= u(k) <= umax];
    
    % Minimize deviation from desired velocity (worst-case robust)
    objective = objective + max((v(k+1) - v_desired)^2);
end

% Set up and solve robust MPC optimization
ops = sdpsettings('verbose', 1, 'solver', 'quadprog');
optimize(constraints, objective, ops);

% Extract optimal control input
u_opt = value(u);
v_opt = value(v);

% Display results
disp('Optimal control inputs:');
disp(u_opt);

disp('Vehicle velocities:');
disp(v_opt);

% Plot the results
figure;
subplot(2,1,1);
stairs(u_opt, 'LineWidth', 2);
title('Optimal Control Inputs (Acceleration)');
xlabel('Time Step'); ylabel('Acceleration (m/s²)');

subplot(2,1,2);
plot(v_opt, 'LineWidth', 2);
hold on;
yline(v_desired, '--r', 'LineWidth', 2);
title('Vehicle Velocity');
xlabel('Time Step'); ylabel('Velocity (m/s)');
legend('Optimal Velocity', 'Desired Velocity');
grid on;
