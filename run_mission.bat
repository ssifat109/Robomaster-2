@echo off
setlocal
title RoboMaster Assessment 2 mission

rem One-click WSL launcher for the supervised Tag 3 mission.
rem The driver remains disarmed unless hardware.yaml says calibration_verified: true.
rem Keep a person beside the robot and keep the emergency stop ready.

set "DISTRO=Ubuntu-24.04"
set "ROOT=/mnt/d/RB 2"

echo Checking WSL, ROS 2, RoboMaster route, and calibration gate...
echo.
echo IMPORTANT: calibration_verified must be true before the robot can move.
echo Current config: %ROOT%/rm_flatness_nav/config/hardware.yaml
wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; command -v ros2 >/dev/null || exit 10; ip route get 192.168.2.1 | grep -q 'src 192.168.2.' || { echo 'ERROR: RoboMaster Wi-Fi route is unavailable.'; exit 11; }; python -c \"import yaml; c=yaml.safe_load(open('%ROOT%/rm_flatness_nav/config/hardware.yaml')); assert c.get('calibration_verified') is True, 'calibration_verified is false'; print('Calibration gate: VERIFIED')\""
if errorlevel 1 (
  echo.
  echo Mission refused. Complete independent camera calibration and range checks,
  echo then set calibration_verified: true in rm_flatness_nav/config/hardware.yaml.
  pause
  exit /b 1
)

echo Running stationary Tag 3 preflight. No chassis motion is commanded.
wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; timeout 35 python '%ROOT%/scripts/sensor_tag_preflight.py' --duration 6"
if errorlevel 1 (
  echo Preflight failed. Mission not started.
  pause
  exit /b 1
)

echo Starting the hardware mission. The chassis is still disarmed during startup.
wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; OUT='%ROOT%/data/hardware_$(date +%%Y%%m%%d_%%H%%M%%S)'; mkdir -p \"$OUT\"; trap 'ros2 service call /rm/stop std_srvs/srv/Trigger {} >/dev/null 2>&1 || true; kill -INT $PID 2>/dev/null || true' INT TERM EXIT; ros2 launch rm_flatness_nav mission.launch.py sim:=false config_file='%ROOT%/rm_flatness_nav/config/hardware.yaml' output_dir=\"$OUT\" >\"$OUT/mission.log\" 2>&1 & PID=$!; echo \"Mission log: $OUT/mission.log\"; sleep 10; ros2 service call /rm/arm std_srvs/srv/SetBool '{data: true}'; wait $PID"

echo Mission process ended. Use run_stop_mission.bat if a stop command is needed.
pause
endlocal
