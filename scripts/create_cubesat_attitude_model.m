% Script to create a Simulink model for CubeSat attitude control (Custom 6DOF)
% Creates cubesat_attitude_custom.slx in the CubeSat_Simulation project

% Define project root and model name
project_root = 'C:\Users\goodking\Documents\MATLAB\CubeSat_Simulation';
model_name = 'cubesat_attitude_custom';
model_path = fullfile(project_root, [model_name, '.slx']);

% Check if Simulink is available
if ~exist('simulink', 'file')
    error('Simulink is required to create the model.');
end

% Create a new Simulink model
if exist(model_path, 'file')
    fprintf('Deleting existing model at %s\n', model_path);
    delete(model_path);
end
model = new_system(model_name);
fprintf('Created new Simulink model: %s\n', model_name);
open_system(model);

% Set model parameters
set_param(model, 'Solver', 'ode45', 'StopTime', '1000');
set_param(model, 'EnableLBRepository', 'on');
fprintf('Set model parameters: Solver=ode45, StopTime=1000\n');

% --- Add Blocks with Error Handling ---

% CubeSat Parameters (Inertia Matrix)
Ixx = 0.008; Iyy = 0.008; Izz = 0.012;
J = diag([Ixx, Iyy, Izz]);
J_inv = inv(J);
try
    add_block('simulink/Sources/Constant', [model '/CubeSat_Inertia_Inv'], ...
        'Value', sprintf('[%f, %f, %f; %f, %f, %f; %f, %f, %f]', ...
            J_inv(1,1), J_inv(1,2), J_inv(1,3), J_inv(2,1), J_inv(2,2), J_inv(2,3), J_inv(3,1), J_inv(3,2), J_inv(3,3)), ...
        'Position', [50, 100, 100, 130]);
    fprintf('Added block: CubeSat_Inertia_Inv\n');
catch err
    fprintf('Error adding CubeSat_Inertia_Inv: %s\n', err.message);
    rethrow(err);
end

% Initial Conditions
q0 = [1; 0; 0; 0];
w0 = [0; 0; 0];
try
    add_block('simulink/Sources/Constant', [model '/Initial_Quaternion'], ...
        'Value', sprintf('[%f; %f; %f; %f]', q0(1), q0(2), q0(3), q0(4)), ...
        'Position', [50, 150, 100, 180]);
    fprintf('Added block: Initial_Quaternion\n');
catch err
    fprintf('Error adding Initial_Quaternion: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Sources/Constant', [model '/Initial_Angular_Velocity'], ...
        'Value', sprintf('[%f; %f; %f]', w0(1), w0(2), w0(3)), ...
        'Position', [50, 200, 100, 230]);
    fprintf('Added block: Initial_Angular_Velocity\n');
catch err
    fprintf('Error adding Initial_Angular_Velocity: %s\n', err.message);
    rethrow(err);
end

% Desired Quaternion (45° yaw at t=500s)
try
    add_block('simulink/Sources/Step', [model '/Desired_Quaternion_Step'], ...
        'Time', '500', 'Before', '[1; 0; 0; 0]', 'After', '[0.9239; 0; 0; 0.3827]', ...
        'Position', [50, 250, 100, 280]);
    fprintf('Added block: Desired_Quaternion_Step\n');
catch err
    fprintf('Error adding Desired_Quaternion_Step: %s\n', err.message);
    rethrow(err);
end

% Custom 6DOF Dynamics
% Angular Acceleration: J^-1 * (tau - w x (J*w))
try
    add_block('simulink/Math Operations/Matrix Multiply', [model '/Jw'], ...
        'Position', [200, 200, 230, 230]);
    fprintf('Added block: Jw\n');
catch err
    fprintf('Error adding Jw: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Math Operations/Product', [model '/w_cross_Jw'], ...
        'Inputs', '***', 'Position', [250, 200, 280, 230]);
    fprintf('Added block: w_cross_Jw\n');
catch err
    fprintf('Error adding w_cross_Jw: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Math Operations/Subtract', [model '/Tau_minus_wJw'], ...
        'Inputs', '--', 'Position', [300, 200, 330, 230]);
    fprintf('Added block: Tau_minus_wJw\n');
catch err
    fprintf('Error adding Tau_minus_wJw: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Math Operations/Matrix Multiply', [model '/Angular_Accel'], ...
        'Position', [350, 200, 380, 230]);
    fprintf('Added block: Angular_Accel\n');
catch err
    fprintf('Error adding Angular_Accel: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Continuous/Integrator', [model '/Integrator_w'], ...
        'InitialCondition', 'Initial_Angular_Velocity', 'Position', [400, 200, 430, 230]);
    fprintf('Added block: Integrator_w\n');
catch err
    fprintf('Error adding Integrator_w: %s\n', err.message);
    rethrow(err);
end

% Quaternion Dynamics: dq/dt = 0.5 * [0 w] * q
try
    add_block('simulink/User-Defined Functions/MATLAB Function', [model '/Omega_Matrix'], ...
        'Position', [200, 150, 230, 180]);
    set_param([model '/Omega_Matrix'], 'MATLABFunction', ...
        'function Omega = fcn(w)\n' + ...
        'Omega = [0, -w(1), -w(2), -w(3); w(1), 0, -w(3), w(2); w(2), w(3), 0, -w(1); w(3), -w(2), w(1), 0];\n' + ...
        'Omega = 0.5 * Omega;');
    fprintf('Added block: Omega_Matrix\n');
catch err
    fprintf('Error adding Omega_Matrix: %s\n', err.message);
    rethrow(err);
end

try
    add_block('simulink/Math Operations/Matrix Multiply', [model '/Quaternion_Dynamics'], ...
        'Position', [250, 150, 280, 180]);
    fprintf('Added block: Quaternion_Dynamics\n');
catch err
    fprintf('Error adding Quaternion_Dynamics: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Continuous/Integrator', [model '/Integrator_q'], ...
        'InitialCondition', 'Initial_Quaternion', 'Position', [300, 150, 330, 180]);
    fprintf('Added block: Integrator_q\n');
catch err
    fprintf('Error adding Integrator_q: %s\n', err.message);
    rethrow(err);
end

% Quaternion Error
try
    add_block('simulink/Math Operations/Subtract', [model '/Quaternion_Error'], ...
        'Inputs', '--', 'Position', [100, 150, 130, 180]);
    fprintf('Added block: Quaternion_Error\n');
catch err
    fprintf('Error adding Quaternion_Error: %s\n', err.message);
    rethrow(err);
end

% Demux Quaternion Error (4 components: q0, q1, q2, q3)
try
    add_block('simulink/Signal Routing/Demux', [model '/Demux_Quaternion_Error'], ...
        'Outputs', '4', 'Position', [140, 150, 160, 180]);
    fprintf('Added block: Demux_Quaternion_Error\n');
catch err
    fprintf('Error adding Demux_Quaternion_Error: %s\n', err.message);
    rethrow(err);
end

% PID Controllers for q1, q2, q3 (ignore q0 for control)
Kp = 0.01; Kd = 0.05;
try
    add_block('simulink/Continuous/PID Controller', [model '/PID_q1'], ...
        'P', num2str(Kp), 'D', num2str(Kd), 'I', '0', ...
        'Position', [200, 140, 250, 160]);
    fprintf('Added block: PID_q1\n');
catch err
    fprintf('Error adding PID_q1: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Continuous/PID Controller', [model '/PID_q2'], ...
        'P', num2str(Kp), 'D', num2str(Kd), 'I', '0', ...
        'Position', [200, 170, 250, 190]);
    fprintf('Added block: PID_q2\n');
catch err
    fprintf('Error adding PID_q2: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Continuous/PID Controller', [model '/PID_q3'], ...
        'P', num2str(Kp), 'D', num2str(Kd), 'I', '0', ...
        'Position', [200, 200, 250, 220]);
    fprintf('Added block: PID_q3\n');
catch err
    fprintf('Error adding PID_q3: %s\n', err.message);
    rethrow(err);
end

% Mux the PID outputs into a 3x1 torque vector
try
    add_block('simulink/Signal Routing/Mux', [model '/Mux_Torque'], ...
        'Inputs', '3', 'Position', [280, 170, 300, 190]);
    fprintf('Added block: Mux_Torque\n');
catch err
    fprintf('Error adding Mux_Torque: %s\n', err.message);
    rethrow(err);
end

% Scope and Output
try
    add_block('simulink/Sinks/Scope', [model '/Quaternion_Scope'], ...
        'NumInputPorts', '1', 'Position', [400, 150, 450, 200]);
    set_param([model '/Quaternion_Scope'], 'NumInputPorts', '1', 'OpenAtSimulationStart', 'on');
    fprintf('Added block: Quaternion_Scope\n');
catch err
    fprintf('Error adding Quaternion_Scope: %s\n', err.message);
    rethrow(err);
end
try
    add_block('simulink/Sinks/To Workspace', [model '/Quaternion_Output'], ...
        'VariableName', 'q_simulink', 'SaveFormat', 'Array', ...
        'Position', [400, 250, 450, 280]);
    fprintf('Added block: Quaternion_Output\n');
catch err
    fprintf('Error adding Quaternion_Output: %s\n', err.message);
    rethrow(err);
end

% --- Connect Blocks ---
try
    add_line(model, 'CubeSat_Inertia_Inv/1', 'Jw/1');
    add_line(model, 'Integrator_w/1', 'Jw/2');
    add_line(model, 'Integrator_w/1', 'w_cross_Jw/1');
    add_line(model, 'Integrator_w/1', 'w_cross_Jw/2');
    add_line(model, 'Jw/1', 'w_cross_Jw/3');
    add_line(model, 'Mux_Torque/1', 'Tau_minus_wJw/1');
    add_line(model, 'w_cross_Jw/1', 'Tau_minus_wJw/2');
    add_line(model, 'Tau_minus_wJw/1', 'Angular_Accel/1');
    add_line(model, 'CubeSat_Inertia_Inv/1', 'Angular_Accel/2');
    add_line(model, 'Angular_Accel/1', 'Integrator_w/1');

    add_line(model, 'Integrator_w/1', 'Omega_Matrix/1');
    add_line(model, 'Omega_Matrix/1', 'Quaternion_Dynamics/1');
    add_line(model, 'Integrator_q/1', 'Quaternion_Dynamics/2');
    add_line(model, 'Quaternion_Dynamics/1', 'Integrator_q/1');

    add_line(model, 'Desired_Quaternion_Step/1', 'Quaternion_Error/1');
    add_line(model, 'Integrator_q/1', 'Quaternion_Error/2');
    add_line(model, 'Quaternion_Error/1', 'Demux_Quaternion_Error/1');
    add_line(model, 'Demux_Quaternion_Error/2', 'PID_q1/1'); % q1 error
    add_line(model, 'Demux_Quaternion_Error/3', 'PID_q2/1'); % q2 error
    add_line(model, 'Demux_Quaternion_Error/4', 'PID_q3/1'); % q3 error
    add_line(model, 'PID_q1/1', 'Mux_Torque/1');
    add_line(model, 'PID_q2/1', 'Mux_Torque/2');
    add_line(model, 'PID_q3/1', 'Mux_Torque/3');

    add_line(model, 'Integrator_q/1', 'Quaternion_Scope/1');
    add_line(model, 'Integrator_q/1', 'Quaternion_Output/1');
    fprintf('All connections added successfully\n');
catch err
    fprintf('Error adding connections: %s\n', err.message);
    rethrow(err);
end

% --- Save and Open the Model ---
save_system(model, model_path);
fprintf('Model saved to: %s\n', model_path);
open_system(model_path);
disp('Custom Simulink model created and saved as cubesat_attitude_custom.slx');