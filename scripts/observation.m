function [img_cropped, temp, co2] = observation(t_orbit, data_folder)
    % Simulate camera and sensor data
    % Inputs:
    %   t_orbit: Nx1 vector of timestamps (seconds)
    %   data_folder: Absolute path to data folder
    
    % Ensure t_orbit is a column vector
    t_orbit = t_orbit(:);
    
    % Debug: Check size of t_orbit
    disp('Size of t_orbit in observation:');
    disp(size(t_orbit));
    
    % Default image if earth_image.jpg is not found
    img_cropped = zeros(200, 200, 3, 'uint8'); % Black image as fallback
    
    % Try to load the image
    image_path = fullfile(data_folder, 'earth_image.jpg');
    try
        if exist(image_path, 'file')
            img = imread(image_path);
            img_cropped = imcrop(img, [500, 500, 200, 200]); % Simulate FOV
        else
            warning('Image file %s not found. Using blank image.', image_path);
        end
    catch err
        warning('Error loading image %s: %s. Using blank image.', image_path, err.message);
    end
    
    % Simulate sensor data
    temp = 50 * rand(size(t_orbit)) - 50; % Temperature (°C)
    co2 = 400 + 10 * randn(size(t_orbit)); % CO2 (ppm)
    
    % Debug: Check sizes
    disp('Size of temp:');
    disp(size(temp));
    disp('Size of co2:');
    disp(size(co2));
    
    % Ensure temp and co2 are column vectors
    temp = temp(:);
    co2 = co2(:);
end