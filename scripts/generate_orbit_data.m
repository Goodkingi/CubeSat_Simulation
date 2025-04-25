% Script to generate GMAT-like orbit data for CubeSat simulation
% Creates orbit_data.csv with position and velocity in ECI coordinates
clear all; clc;

% Orbital parameters
alt = 500e3; % Altitude (m)
R_earth = 6378e3; % Earth radius (m)
mu = 3.986e14; % Earth's gravitational constant (m^3/s^2)
a = R_earth + alt; % Semi-major axis (m)
inc = deg2rad(51.6); % Inclination (rad)
RAAN = 0; % Right Ascension of Ascending Node (rad)
arg_peri = 0; % Argument of Perigee (rad)
v = 0; % Initial true anomaly (rad)

% Simulation parameters
tspan = 0:1:86400; % 24 hours (s)
n = sqrt(mu/a^3); % Mean motion (rad/s)

% Initialize arrays
pos = zeros(length(tspan), 3); % Position (x, y, z) in ECI (m)
vel = zeros(length(tspan), 3); % Velocity (vx, vy, vz) in ECI (m/s)

% Compute orbit
for i = 1:length(tspan)
    % True anomaly at time t
    theta = v + n * tspan(i);
    
    % Position in perifocal frame
    r = a; % Circular orbit
    r_perifocal = [r * cos(theta); r * sin(theta); 0];
    v_perifocal = [-r * n * sin(theta); r * n * cos(theta); 0];
    
    % Convert to ECI frame
    % Rotation matrix: Perifocal to ECI
    R = rotm_eci_perifocal(RAAN, inc, arg_peri);
    pos(i,:) = (R * r_perifocal)';
    vel(i,:) = (R * v_perifocal)';
end

% Save to CSV
data = table(tspan', pos(:,1), pos(:,2), pos(:,3), vel(:,1), vel(:,2), vel(:,3), ...
    'VariableNames', {'time', 'x', 'y', 'z', 'vx', 'vy', 'vz'});
writetable(data, '../data/orbit_data.csv');
disp('orbit_data.csv saved to data/ folder.');

% Helper function: Rotation matrix from perifocal to ECI
function R = rotm_eci_perifocal(RAAN, inc, arg_peri)
    % Rotation matrix: Perifocal to ECI
    R1 = [cos(RAAN), -sin(RAAN), 0; sin(RAAN), cos(RAAN), 0; 0, 0, 1];
    R2 = [1, 0, 0; 0, cos(inc), -sin(inc); 0, sin(inc), cos(inc)];
    R3 = [cos(arg_peri), -sin(arg_peri), 0; sin(arg_peri), cos(arg_peri), 0; 0, 0, 1];
    R = R1 * R2 * R3;
end