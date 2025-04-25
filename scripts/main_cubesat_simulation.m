% Main CubeSat Simulation Script
% Simulates a 3U CubeSat in LEO for environmental observation, GPS tracking,
% and communication with a ground station.
clear all; clc; close all;

% Define project root directory
project_root = 'C:\Users\goodking\Documents\MATLAB\CubeSat_Simulation';
data_folder = fullfile(project_root, 'data');
results_folder = fullfile(project_root, 'results');

% Create results folder if it doesn't exist
if ~exist(results_folder, 'dir')
    mkdir(results_folder);
end

% Add scripts folder to path
addpath(fullfile(project_root, 'scripts'));

% --- Initialize Parameters ---
% CubeSat properties
mass = 4; % kg
Ixx = 0.008; Iyy = 0.008; Izz = 0.012; % Moment of inertia (kg*m^2)
J = diag([Ixx, Iyy, Izz]); % Inertia matrix

% Orbital parameters
alt = 500e3; % Altitude (m)
mu = 3.986e14; % Earth's gravitational constant (m^3/s^2)
R_earth = 6378e3; % Earth radius (m)
a = R_earth + alt; % Semi-major axis (m)
n = sqrt(mu/a^3); % Mean motion (rad/s)

% Simulation parameters
tspan = 0:1:1000; % 1000 seconds for testing
q0 = [1; 0; 0; 0]; % Initial quaternion (nadir pointing)
w0 = [0; 0; 0]; % Initial angular velocity (rad/s)

% Ground station coordinates
gs_lla = [0, 0, 0]; % Lat, Lon, Alt (deg, deg, m)

% --- Load Orbit Data from GMAT ---
try
    orbit_data = readtable(fullfile(data_folder, 'orbit_data.csv'));
    % Check if expected columns exist
    if ~all(ismember({'time', 'x', 'y', 'z'}, orbit_data.Properties.VariableNames))
        error('orbit_data.csv does not contain required columns: time, x, y, z');
    end
    t_orbit = orbit_data.time;
    pos_eci = [orbit_data.x, orbit_data.y, orbit_data.z]; % ECI position (m)
    % Truncate to match tspan
    idx = t_orbit <= max(tspan);
    t_orbit = t_orbit(idx);
    pos_eci = pos_eci(idx,:);
catch err
    warning('Failed to load orbit_data.csv: %s. Using simplified orbit model.', err.message);
    t_orbit = tspan;
    pos_eci = zeros(length(tspan), 3);
    for i = 1:length(tspan)
        theta = n * tspan(i);
        pos_eci(i,:) = a * [cos(theta), sin(theta), 0]; % Simplified ECI
    end
end

% Ensure t_orbit is a column vector
t_orbit = t_orbit(:);

% Debug: Check sizes
disp('Size of pos_eci:');
disp(size(pos_eci));
disp('Length of t_orbit:');
disp(length(t_orbit));

% --- Attitude Dynamics ---
try
    Kp = 0.01; Kd = 0.05; % PID gains
    [t, state] = ode45(@(t, state) attitude_dynamics(t, state, J, Kp, Kd, n), tspan, [q0; w0]);
    q = state(:,1:4); % Quaternion

    % Plot attitude
    figure;
    plot(t, q);
    xlabel('Time (s)'); ylabel('Quaternion');
    title('CubeSat Attitude');
    legend('q0', 'q1', 'q2', 'q3');
    saveas(gcf, fullfile(results_folder, 'attitude_plot.png'));
catch err
    warning('Error in attitude dynamics: %s', err.message);
end

% --- Convert ECI to ECEF for GPS and Communication ---
try
    % Earth's angular velocity (rad/s)
    omega_earth = 7.292115e-5;
    pos_ecef = zeros(size(pos_eci));
    for i = 1:size(pos_eci, 1)
        % Rotation angle due to Earth's rotation
        theta = omega_earth * t_orbit(i);
        % ECI to ECEF rotation matrix (rotation around z-axis)
        R = [cos(theta), sin(theta), 0; -sin(theta), cos(theta), 0; 0, 0, 1];
        pos_ecef(i,:) = (R * pos_eci(i,:)')';
    end
catch err
    warning('Error converting ECI to ECEF: %s', err.message);
    pos_ecef = pos_eci; % Fallback (may cause issues in GPS/communication)
end

% --- GPS Tracking ---
try
    [lla, vel] = gps_tracking(pos_ecef, t_orbit);
    figure;
    geoplot(lla(:,1), lla(:,2), 'b.');
    geobasemap('satellite');
    title('CubeSat Ground Track');
    saveas(gcf, fullfile(results_folder, 'ground_track.png'));
catch err
    warning('Error in GPS tracking: %s', err.message);
end

% --- Environmental Observation ---
try
    [img_cropped, temp, co2] = observation(t_orbit, data_folder);
    figure;
    imshow(img_cropped);
    title('Simulated Camera Image');
    saveas(gcf, fullfile(results_folder, 'camera_image.png'));

    % Save sensor data
    output_folder = fullfile(data_folder, 'output');
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
    end
    sensor_data = table(t_orbit, temp, co2, 'VariableNames', {'Time_s', 'Temperature_C', 'CO2_ppm'});
    writetable(sensor_data, fullfile(output_folder, 'sensor_data.csv'));
catch err
    warning('Error in observation: %s', err.message);
end

% --- Communication Simulation ---
try
    [data_transmitted] = communication(t_orbit, pos_ecef, gs_lla);
    figure;
    plot(t_orbit, data_transmitted/1e6);
    xlabel('Time (s)'); ylabel('Data Transmitted (MB)');
    title('Data Transmission');
    saveas(gcf, fullfile(results_folder, 'communication_plot.png'));
catch err
    warning('Error in communication: %s', err.message);
end

disp('Simulation complete. Results saved in results/ folder.');