@echo off
wsl.exe -d Ubuntu-24.04 --cd "%~dp0." -- /home/ugo/.venvs/heavy-arm/bin/python -B demo_pinocchio.py
pause
