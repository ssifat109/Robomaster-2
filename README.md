# RoboMaster differential-flatness pursuit

ROS 2 Jazzy package for SCE 672 Assessment 2. The project targets a RoboMaster EP from Ubuntu 24.04 under WSL 2 mirrored networking.

## Package contents

- `rm_flatness_nav/`: planner, SDK driver, camera/AprilTag processing, voltage-model logger, simulator, launch file, configuration, and tests.
- `scripts/`: WSL environment, build/test helper, stationary hardware checks, calibration tools, simulation runner, and evidence analysis.
- `report_source/`: LaTeX report source. The final hardware telemetry and YouTube link must be inserted after the supervised trial.

## WSL build and test

```bash
bash scripts/build_test.sh
```

The helper expects ROS 2 Jazzy in `/opt/ros/jazzy`, a Python environment at `~/rm_venv`, and creates or updates `~/rb2_ws`.

## Simulation

```bash
source scripts/env.sh
python scripts/run_simulation.py data/simulation_run
python scripts/check_simulation.py data/simulation_run
python scripts/analyze_run.py data/simulation_run
```

Simulation CSV files are labelled `SIMULATION` and must not be presented as experimental data.

## Hardware use

Keep this repository private. Connect Windows to the RoboMaster access point and use WSL mirrored networking. Complete stationary camera calibration and independent range checks before changing `calibration_verified` to `true`. The driver starts disarmed and requires a fresh Tag 3 observation before arming.

The assessment voltage values come from the required wheel-rate and armature-voltage equations. They are model estimates based on acknowledged commands, not direct terminal-voltage measurements.

## Links to add before presentation

- Private GitHub URL: `ADD AFTER UPLOAD`
- YouTube demonstration: `ADD AFTER UPLOAD`
