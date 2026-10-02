@echo off
setlocal EnableDelayedExpansion
title Vivado build - freq-lab VLF SID receiver

REM ================================================================
REM  Vivado batch build of the VLF solar-flare receiver (vlf_sid)
REM  PZ7020-StarLite (xc7z020clg400-2). Double-click to run.
REM  Progress: build_status_vlf.txt    Vivado log: log_vlf_sid.log
REM ================================================================

set "FPGA_DIR=%~dp0"
if "%FPGA_DIR:~-1%"=="\" set "FPGA_DIR=%FPGA_DIR:~0,-1%"
set "STATUS=%FPGA_DIR%\build_status_vlf.txt"

REM ---- license: Vivado reads XILINXD_LICENSE_FILE if your setup needs one; the XC7Z020 builds with the free edition ----

set "VIVADO="
REM ---- find vivado.bat: VIVADO_BAT if you set it, then Vivado's own XILINX_VIVADO, then vivado.bat on PATH ----
REM      (not found? run this from a Vivado command prompt, or set VIVADO_BAT to the full path of your vivado.bat)
if defined VIVADO_BAT if exist "%VIVADO_BAT%" set "VIVADO=%VIVADO_BAT%"
if not defined VIVADO if defined XILINX_VIVADO if exist "%XILINX_VIVADO%\bin\vivado.bat" set "VIVADO=%XILINX_VIVADO%\bin\vivado.bat"
if not defined VIVADO for /f "delims=" %%V in ('where vivado.bat 2^>nul') do if not defined VIVADO set "VIVADO=%%V"

if not defined VIVADO (
    echo ERROR: could not find vivado.bat
    echo ERROR: vivado.bat not found in any known location> "%STATUS%"
    echo COMPLETED: aborted, Vivado not found>> "%STATUS%"
    pause
    exit /b 1
)

echo Using Vivado: %VIVADO%
echo Vivado: %VIVADO%> "%STATUS%"
echo License: %XILINXD_LICENSE_FILE%>> "%STATUS%"
echo Started: %date% %time%>> "%STATUS%"

echo.
echo BUILDING VLF_SID ...
echo vlf_sid: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\vlf_sid"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_vlf_sid.log" -journal "%FPGA_DIR%\jou_vlf_sid.jou"
set "EC=!errorlevel!"
echo vlf_sid: EXIT_CODE=!EC! at %time%>> "%STATUS%"

echo COMPLETED: %date% %time%>> "%STATUS%"
echo.
echo ================================================================
echo  VLF_SID BUILD FINISHED (exit code !EC!) - see build_status_vlf.txt
echo  Bitstream: %FPGA_DIR%\vlf_sid\build\vlf_sid.bit
echo  (this window can be closed)
echo ================================================================
pause
