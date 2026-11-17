@echo off
setlocal
title RoboMaster Tag 3 recording panel
wsl -d Ubuntu-24.04 -- bash -lc "bash '/mnt/d/RB 2/scripts/start.sh'"
echo.
echo Panel closed. The mission should be stopped.
pause
