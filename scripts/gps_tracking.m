function [lla, vel] = gps_tracking(pos_ecef, t_orbit)
    % Simulate GPS tracking
    % Inputs:
    %   pos_ecef: Nx3 array of ECEF positions (meters)
    %   t_orbit: Nx1 vector of timestamps (seconds since simulation start)
    
    % Validate inputs
    if size(pos_ecef, 2) ~= 3
        error('Position input must be an Nx3 array (x, y, z).');
    end
    if length(t_orbit) ~= size(pos_ecef, 1)
        error('Length of t_orbit (%d) must match number of rows in pos_ecef (%d).', ...
            length(t_orbit), size(pos_ecef, 1));
    end
    
    % Debug: Check size of pos_ecef
    disp('Size of pos_ecef in gps_tracking:');
    disp(size(pos_ecef));
    
    % Convert ECEF to LLA directly using ecef2lla
    lla = zeros(size(pos_ecef));
    for i = 1:size(pos_ecef, 1)
        lla(i,:) = ecef2lla(pos_ecef(i,:));
    end
    
    % Add simulated GPS noise (e.g., 10-meter equivalent noise in LLA)
    noise_std = 0.0001; % Roughly 10 meters in degrees (1 deg ≈ 111 km)
    lla(:,1:2) = lla(:,1:2) + noise_std * randn(size(lla(:,1:2))); % Noise on lat/lon
    lla(:,3) = lla(:,3) + 10 * randn(size(lla(:,3))); % Noise on altitude (meters)
    
    % Velocity (not simulated, set to zeros)
    vel = zeros(size(lla));
    
    % Debug: Check size of lla
    disp('Size of lla in gps_tracking:');
    disp(size(lla));
end