@echo off
setlocal EnableDelayedExpansion
title Vivado build - freq-lab VLF SID receiver v2 (sferic capture)

REM ================================================================
REM  Vivado batch build of the VLF solar-flare receiver, version 2
REM  (adds lightning time-of-arrival capture). PZ7020-StarLite.
REM  Progress: build_status_vlf_v2.txt    Vivado log: log_vlf_sid_v2.log
REM ================================================================

set "FPGA_DIR=%~dp0"
if "%FPGA_DIR:~-1%"=="\" set "FPGA_DIR=%FPGA_DIR:~0,-1%"
set "STATUS=%FPGA_DIR%\build_status_vlf_v2.txt"

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
echo BUILDING VLF_SID_V2 ...
echo vlf_sid_v2: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\vlf_sid_v2"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_vlf_sid_v2.log" -journal "%FPGA_DIR%\jou_vlf_sid_v2.jou"
set "EC=!errorlevel!"
echo vlf_sid_v2: EXIT_CODE=!EC! at %time%>> "%STATUS%"

echo COMPLETED: %date% %time%>> "%STATUS%"
echo.
echo ================================================================
echo  VLF_SID_V2 BUILD FINISHED (exit code !EC!) - see build_status_vlf_v2.txt
echo  Bitstream: %FPGA_DIR%\vlf_sid_v2\build\vlf_sid_v2.bit
echo  (this window can be closed)
echo ================================================================
if "%1"=="" pause
