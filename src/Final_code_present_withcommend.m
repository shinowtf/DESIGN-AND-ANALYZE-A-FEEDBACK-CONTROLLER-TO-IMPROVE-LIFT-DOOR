clc;
clear;

% Door Parameters
door_weight = 100; % kg (for one side)
door_width = 1;  % m (maximum opening distance)
time_to_open = 5;  % seconds (time to open fully)
disp(['Total weight of door 1 + door 2 = ', num2str(door_weight), ' kg']);
disp(['Total width door open ', num2str(door_width), ' m']);
% Gravitational force acting on door
g = 9.81; % m/s^2
force = door_weight * g; % N
disp(['Total force needed by motor ', num2str(force), ' N']);

% Torque required to overcome door weight (assuming radius of gear is 0.2 m)
radius = 0.2; % m
required_torque = force * radius; % Nm
%gear torque open door
disp(['Required torque needed by gear for open door ', num2str(required_torque), ' Nm  ']);

% Motor Parameters
J = 0.0000459;      % Rotor Inertia (kg·m^2)
B = 0.000009549;    % Damping Coefficient (N·m·s/rad)
K = 0.3065;         % Motor Constant (N·m/A or V·s/rad)
R = 16.5;           % Armature Resistance (Ohms)
L = 0.027;          % Armature Inductance (H)
b = 0.000009549;    % Damping Coefficient (Nms/rad)

% Door Dynamics
door_inertia = (1/3) * door_weight * door_width^2; 
total_inertia = J + door_inertia * radius^2;       % Combined system inertia
disp(['Total Inertia ', num2str(total_inertia), ' kg m^2']);
% Simulation Parameters
sim_time = 10; % Simulation time in seconds
ts = 0.001;    % Time step (s)
t = 0:ts:sim_time;

% PID Parameters
Kp = 0.340882457274351;
Ki = 39.3793860956336;
Kd = 0.000400073028738711;

% Preallocate Arrays
position_no_pid = zeros(size(t));
velocity_no_pid = zeros(size(t));
torque_no_pid = zeros(size(t));

position_pid = zeros(size(t));
velocity_pid = zeros(size(t));
torque_pid = zeros(size(t));

% Initial Conditions (No PID)
theta_no_pid = 0;    % Initial position (rad)
omega_no_pid = 0;    % Initial angular velocity (rad/s)
i_a_no_pid = 0;      % Initial armature current (A)

% Initial Conditions (With PID)
theta_pid = 0;    % Initial position (rad)
omega_pid = 0;    % Initial angular velocity (rad/s)
i_a_pid = 0;      % Initial armature current (A)
integral = 0;     % Integral term initialization
previous_error = 0; % Previous error initialization

% Add noise types to simulate real-world conditions
sensor_noise_std = 0.1; % Standard deviation for sensor noise
torque_noise_std = 0.2; % Standard deviation for torque disturbances
voltage_noise_std = 0.5; % Standard deviation for voltage noise
position_noise_std = 0.010;

% Simulate Motor Dynamics without PID (with noise)
for k = 1:length(t)
    % Add random noise to torque
    torque_noise = torque_noise_std * randn;
    torque_no_pid(k) = K * i_a_no_pid + torque_noise;

    % Update Angular Velocity (using torque and damping)
    alpha_no_pid = (torque_no_pid(k) - B * omega_no_pid) / total_inertia;
    omega_no_pid = omega_no_pid + alpha_no_pid * ts;

    % Add random noise to angular velocity (sensor noise)
    omega_no_pid_measured = omega_no_pid + sensor_noise_std * randn;

    % Update Angular Position (integration)
    theta_no_pid = theta_no_pid + omega_no_pid_measured * ts;
    position_no_pid(k) = theta_no_pid * radius;

    % Saturate the Position
    if position_no_pid(k) > door_width
        position_no_pid(k) = door_width;
    elseif position_no_pid(k) < 0
        position_no_pid(k) = 0;
    end
    position_no_pid(k) = position_no_pid(k) + position_noise_std * randn;

    % Calculate Back EMF
    V_b_no_pid = K * omega_no_pid;

    % Add noise to input voltage (voltage fluctuations)
    voltage_noise = voltage_noise_std * randn;
    i_a_dot_no_pid = (12 + voltage_noise - V_b_no_pid - R * i_a_no_pid) / L;
    i_a_no_pid = i_a_no_pid + i_a_dot_no_pid * ts;

    % Store Velocity
    velocity_no_pid(k) = omega_no_pid_measured;
end

% Simulate Motor Dynamics with PID (with noise)
for k = 1:length(t)
    % Calculate Error for Kp
    error = door_width - position_pid(k);
    
    % PID Control Law
    integral = integral + error * ts;
    derivative = (error - previous_error) / ts;
    control_signal = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    
    % Saturate Control Signal (Voltage)
    V_pid = max(min(control_signal + voltage_noise_std * randn, 24), 0);

    % Add random noise to torque
    torque_noise = torque_noise_std * randn;
    torque_pid(k) = K * i_a_pid + torque_noise;

    % Update Angular Velocity (using torque and damping)
    alpha_pid = (torque_pid(k) - B * omega_pid) / total_inertia;
    omega_pid = omega_pid + alpha_pid * ts;

    % Add random noise to angular velocity (sensor noise)
    omega_pid_measured = omega_pid + sensor_noise_std * randn;

    % Update Angular Position (integration)
    theta_pid = theta_pid + omega_pid_measured * ts;
    position_pid(k) = theta_pid * radius;

    % Saturate the Position
    if position_pid(k) > door_width
        position_pid(k) = door_width;
    elseif position_pid(k) < 0
        position_pid(k) = 0;
    end
    position_pid(k) = position_pid(k) + position_noise_std * randn;

    % Calculate Back EMF
    V_b_pid = K * omega_pid;

    % Calculate Armature Current
    i_a_dot_pid = (V_pid - V_b_pid - R * i_a_pid) / L;
    i_a_pid = i_a_pid + i_a_dot_pid * ts;

    % Store Velocity
    velocity_pid(k) = omega_pid_measured;
end


% Calculate Time to Fully Open
%find func will find the first index position_no_pid that door greater than
%door_width
time_no_pid = find(position_no_pid >= door_width, 1) * ts; % Time without PID
%>= find from a to b, find 1meter(door reaches 1meter)
time_with_pid = find(position_pid >= door_width, 1) * ts;  % Time with PID

% Display Results
disp(['Time to fully open the door without PID: ', num2str(time_no_pid), ' seconds']);
disp(['Time to fully open the door with PID: ', num2str(time_with_pid), ' seconds']);

% Transfer Function for Motor (Step Response)
numerator = [K]; % Numerator of the transfer function
denominator = [L*J, (R*J + B*L), (R*B + K^2)]; % Denominator of the transfer function
motor_tf = tf(numerator, denominator); % Define transfer function using tf

% Step Response Simulation
[t_step, step_response_values] = step(12 * motor_tf, 20); % Step response for a 20-second simulation

% Motor Angular Velocity vs Time (with gradual decrease and noise)
velocity_pid_with_decay = velocity_pid;
decay_start_idx = find(position_pid >= 0.58*door_width, 1); % Find the index when the door is fully open

if ~isempty(decay_start_idx)
    for k = decay_start_idx:length(t)
        % Gradual decrease in angular velocity
        decay_factor = exp(-(t(k) - t(decay_start_idx)) * 2); % Exponential decay factor
        noise = 0.03 * randn; % Small random noise
        velocity_pid_with_decay(k) = velocity_pid_with_decay(decay_start_idx) * decay_factor + noise;
    end
end

%tst
velocity_pid_without_decay = velocity_no_pid;
decay_start_idxs = find(position_no_pid >= 0.58*door_width, 1); % Find the index when the door is fully open

if ~isempty(decay_start_idxs)
    for k = decay_start_idxs:length(t)
        % Gradual decrease in angular velocity
        decay_factor = exp(-(t(k) - t(decay_start_idxs)) * 2); % Exponential decay factor
        noise = 0.03 * randn; % Small random noise
        velocity_pid_without_decay(k) = velocity_pid_without_decay(decay_start_idxs) * decay_factor + noise;
    end
end




% Adjusting the plot for Angular Velocity
figure;
% Door Position vs Time
subplot(2,1,1);
plot(t, position_no_pid, 'b', 'LineWidth', 2);
hold on;
plot(t, position_pid, 'r', 'LineWidth', 2);
title('Door Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
legend('No PID', 'With PID');
grid on;

% Motor Angular Velocity vs Time
subplot(2,1,2);
plot(t, velocity_pid_without_decay, 'b', 'LineWidth', 2);
hold on;
plot(t, velocity_pid_with_decay, 'r', 'LineWidth', 2);
title('Motor Angular Velocity vs Time (With Decay and Noise)');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('No PID', 'With PID');
grid on;

