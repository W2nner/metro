@echo off
cd /d "%~dp0"
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=1
if exist "..\lidar-review" (
 python -m metro_detector serve --data "..\lidar-review" --state state --port 8190
) else (
 python -m metro_detector serve --state state --port 8190
)
pause
