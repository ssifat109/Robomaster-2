# RoboMaster Assessment 2 submission

Student ID: 202613880. This repository contains the ROS 2 Jazzy package, WSL scripts, report source and PDF, assignment brief, and available robot evidence.

## Evidence notes

- `data/hardware_run_1/` contains the completed camera-guided Tag 3 approach logs and robot-camera MP4. The final camera range was approximately 0.24 m; the planner logged `STOPPED_SHORT` at its 0.20 m nominal threshold.
- `data/hardware_run_2_attempt/` contains a separate attempt's logs. Do not combine its data with the completed run.
- `evidence/screen_recordings/control_panel_20261118.mp4` is the supplied control panel screen recording. The student phone video should be added separately if required.
- Motor voltages in the CSV are model estimates derived from commanded wheel speeds, not direct motor voltage measurements. ToF returned zero readings in the available sessions.

## Run on Windows + WSL

Use Ubuntu 24.04 with ROS 2 Jazzy. `run_mission.bat` launches the mission workflow and `run_stop_mission.bat` sends a stop request. Review `rm_flatness_nav/config/hardware.example.yaml` and the report before use.
