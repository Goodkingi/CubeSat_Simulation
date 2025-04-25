# CubeSat Simulation Project
Simulates a 3U CubeSat in LEO for environmental observation, GPS tracking, and ground station communication.

## Requirements
- MATLAB R2023b with:
  - Aerospace Toolbox
  - Satellite Communications Toolbox
  - Navigation Toolbox
  - Image Processing Toolbox
  - Control System Toolbox
  - Mapping Toolbox
- GMAT (for orbit data)
- Sample Earth image (e.g., NASA's Blue Marble)

## Setup
1. Place `orbit_data.csv` from GMAT in `data/`.
2. Download an Earth image and save as `data/earth_image.jpg`.
3. Run `main_cubesat_simulation.m` in MATLAB.

## Folder Structure
- `data/`: Orbit data, sample image, and outputs.
- `scripts/`: MATLAB scripts for simulation.
- `simulink/`: Simulink models (e.g., attitude control).
- `doc/`: Documentation.
- `results/`: Output plots and data.

## Outputs
- Plots: Ground track, attitude, camera image, communication.
- Data: Sensor data CSV in `data/output/`.