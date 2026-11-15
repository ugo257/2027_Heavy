@echo off
wsl.exe -d Ubuntu-24.04 --cd "%~dp0." -- /home/ugo/.venvs/heavy-arm/bin/python -B sim/viewer.py
if errorlevel 1 pause
