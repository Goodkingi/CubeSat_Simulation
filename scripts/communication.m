function [data_transmitted] = communication(t_orbit, pos_ecef, gs_lla)
    % Simulate S-band communication
    % Inputs:
    %   t_orbit: Nx1 vector of timestamps (seconds)
    %   pos_ecef: Nx3 array of ECEF positions (meters)
    %   gs_lla: 1x3 vector of ground station coordinates (lat, lon, alt in deg, deg, m)
    
    % Validate inputs
    if nargin < 3
        error('Not enough input arguments. Expected t_orbit, pos_ecef, and gs_lla.');
    end
    if size(pos_ecef, 2) ~= 3
        error('Position input must be an Nx3 array (x, y, z).');
    end
    if length(t_orbit) ~= size(pos_ecef, 1)
        error('Length of t_orbit (%d) must match number of rows in pos_ecef (%d).', ...
            length(t_orbit), size(pos_ecef, 1));
    end
    if length(gs_lla) ~= 3
        error('gs_lla must be a 1x3 vector (lat, lon, alt).');
    end
    
    % Debug: Confirm inputs
    disp('Running communication simulation...');
    disp('Size of pos_ecef in communication:');
    disp(size(pos_ecef));
    disp('Length of t_orbit in communication:');
    disp(length(t_orbit));
    
    % Simulate communication
    elevation = zeros(size(t_orbit));
    for i = 1:length(t_orbit)
        try
            sat_lla = ecef2lla(pos_ecef(i,:)); % pos_ecef is already in meters
            [~, el, ~] = geodetic2aer(sat_lla(1), sat_lla(2), sat_lla(3), gs_lla(1), gs_lla(2), gs_lla(3));
            elevation(i) = el;
        catch
            elevation(i) = 0; % Fallback if conversion fails
        end
    end
    link_active = elevation > 10; % Link active if elevation > 10°
    data_rate = 1e6; % 1 Mbps
    data_transmitted = cumsum(link_active) * data_rate / 8; % Bytes
end