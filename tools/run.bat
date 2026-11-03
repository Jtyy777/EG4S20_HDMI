@echo off
rem ===================================================================
rem  bmp4fpga.py launcher
rem  Finds a Python that has Pillow, then runs bmp4fpga.py with the
rem  arguments you gave. Place this .bat next to bmp4fpga.py.
rem
rem  Examples:
rem    run.bat --drives
rem    run.bat D:\pics -o E:\
rem    run.bat --check E:\
rem    run.bat --scrub --wipe E:\ --yes
rem ===================================================================
setlocal enabledelayedexpansion
set "SCRIPT=%~dp0bmp4fpga.py"

if not exist "%SCRIPT%" (
    echo [ERROR] bmp4fpga.py not found next to this .bat
    pause
    exit /b 1
)

set "PILPY="
set "ANY="
for %%P in ("D:\Python\python.exe" "python.exe" "py.exe") do (
    %%~P -c "print(1)" >nul 2>&1
    if !errorlevel! equ 0 (
        if not defined ANY set "ANY=%%~P"
        %%~P -c "import PIL" >nul 2>&1
        if !errorlevel! equ 0 if not defined PILPY set "PILPY=%%~P"
    )
)

set "PY="
if defined PILPY set "PY=%PILPY%"
if not defined PY if defined ANY set "PY=%ANY%"

if not defined PY (
    echo [ERROR] No working Python found.
    echo         Install Python 3.8+ first, then:  pip install Pillow
    pause
    exit /b 1
)

if not defined PILPY (
    echo [WARN] This Python has no Pillow.
    echo        Convert and --check will fail. Fix with:  pip install Pillow
    echo        --scrub and --wipe still work.
    echo.
)

"%PY%" "%SCRIPT%" %*
set "RC=%errorlevel%"

if "%~1"=="" pause
exit /b %RC%
