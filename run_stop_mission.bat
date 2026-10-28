@echo off
setlocal
title Stop RoboMaster Assessment 2 mission
wsl -d Ubuntu-24.04 -- bash -lc "source '/mnt/d/RB 2/scripts/env.sh'; timeout 5 ros2 service call /rm/stop std_srvs/srv/Trigger '{}' || true"
echo Stop request sent. The driver latches zero commands.
pause
endlocal
