@echo off
cd /d "%~dp0"
"C:\Users\zhou_\miniconda3\python.exe" -B demo_mujoco.py
if errorlevel 1 pause
