<<<<<<< HEAD
# CubeSat Simulation
This project simulates the attitude control of a CubeSat using MATLAB and Simulink.

## Project Structure
- `scripts/`: Contains MATLAB scripts like `create_cubesat_attitude_model_custom.m`.
- `results/`: Stores simulation results and plots.
- `doc/`: Documentation, including `README.md`.

## Getting Started
1. Clone this repository.
    https://github.com/Goodkingi/CubeSat_Simulation.git
2. Open MATLAB and navigate to the project folder.
3. Run `scripts/create_cubesat_attitude_model_custom.m` to build the Simulink model.
4. Check `results/` for simulation outputs.

## Requirements
- MATLAB R2023b
- Simulink

# CubeSat Simulation

This project simulates the attitude control of a CubeSat using MATLAB and Simulink. It models the CubeSat's attitude dynamics, implements a PID controller, and simulates a 45° yaw maneuver at t = 500 seconds.

## Project Structure
- `scripts/`: Contains MATLAB scripts like `create_cubesat_attitude_model_custom.m`.
- `results/`: Stores simulation results and plots (e.g., `attitude_plot_simulink_custom.png`).
- `doc/`: Documentation, including this `README.md`.

## Getting Started
1. Clone this repository:
2. Open MATLAB and navigate to the project folder.
3. Run `scripts/create_cubesat_attitude_model_custom.m` to build the Simulink model.
4. Check `results/` for simulation outputs.

## Requirements
- MATLAB R2023b
- Simulink

## Simulation Details
- **Objective**: Simulate a 45° yaw maneuver at t = 500 seconds.
- **Initial Conditions**: Quaternion q0 = [1; 0; 0; 0], Angular Velocity w0 = [0; 0; 0].
- **Target Quaternion**: q = [0.9239; 0; 0; 0.3827] (45° yaw).
- **Inertia Matrix**: J = diag([0.008, 0.008, 0.012]).
- **Controller**: PID with Kp = 0.01, Kd = 0.05.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
=======
simulink
>>>>>>> ff6a108386e7054fc387ab3a0214f8fb97685d9a
