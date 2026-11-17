@echo off
setlocal EnableExtensions
title RoboMaster Assessment 2 - 360 sweep, Tag 3 pursuit

set "DISTRO=Ubuntu-24.04"
set "ROOT=/mnt/d/RB 2"
set "CONFIG=%ROOT%/rm_flatness_nav/config/hardware.yaml"
set "STAMP="

echo RoboMaster video mission: Tag 3, one controlled 360-degree sweep, then pursuit.
echo The robot will stop at the 0.20 m camera/ToF guard.
echo.
echo SAFETY: keep the arena clear and keep an emergency stop ready.
echo CAMERA-ONLY MODE: camera PnP distance stops the robot at 0.20 m.
echo ToF is ignored because this setup has no valid ToF reading.
echo.

wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; command -v ros2 >/dev/null; python -c \"import yaml; c=yaml.safe_load(open('%CONFIG%')); assert c.get('calibration_verified') is True, 'calibration_verified is false'\""
if errorlevel 1 (
  echo.
  echo Mission refused. Fix the ToF/camera calibration first, then set calibration_verified: true.
  pause
  exit /b 1
)

echo Running stationary camera/ToF preflight...
wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; python '%ROOT%/scripts/sensor_tag_preflight.py' --duration 6"
if errorlevel 1 (
  echo Preflight failed. No motion commanded.
  pause
  exit /b 1
)

echo Starting ROS 2 mission. The initial sweep lasts 20.95 s at 0.30 rad/s.
wsl -d %DISTRO% -- bash -lc "source '%ROOT%/scripts/env.sh'; rm -rf /tmp/rm_video_run; mkdir -p /tmp/rm_video_run; trap 'ros2 service call /rm/stop std_srvs/srv/SetBool \"{data: true}\" >/dev/null 2>&1 || true' EXIT; ros2 launch rm_flatness_nav mission.launch.py sim:=false initial_sweep_s:=20.95 config_file:='%CONFIG%' output_dir:=/tmp/rm_video_run >/tmp/rm_video_launch.log 2>&1 & PID=\$!; sleep 25; echo 'Arming driver after camera startup and sweep...'; ros2 service call /rm/arm std_srvs/srv/SetBool \"{data: true}\"; wait \$PID; rc=\$?; mkdir -p '%ROOT%/data/hardware_video_latest'; cp -a /tmp/rm_video_run/. '%ROOT%/data/hardware_video_latest/'; cp /tmp/rm_video_launch.log '%ROOT%/data/hardware_video_latest/launch.log'; exit \$rc"

echo Mission ended. Keep the robot stationary and save the generated CSV/log folder.
pause
