@echo off
setlocal EnableDelayedExpansion
title Vivado build - freq-lab VLF SID receiver v2 (sferic capture)

REM ================================================================
REM  Vivado batch build of the VLF solar-flare receiver, version 2
REM  (adds lightning time-of-arrival capture). PZ7020-StarLite.
REM  Progress: build_status_vlf_v2.txt    Vivado log: log_vlf_sid_v2.log
REM ================================================================

set "FPGA_DIR=P:\Downloads\freq-lab\fpga"
set "STATUS=%FPGA_DIR%\build_status_vlf_v2.txt"

REM ---- license: P:\Downloads\Xilinx.lic (Vivado Basic, node-locked) ----
set "LICFILE=P:\Downloads\Xilinx.lic"
if defined XILINXD_LICENSE_FILE (set "XILINXD_LICENSE_FILE=%LICFILE%;%XILINXD_LICENSE_FILE%") else (set "XILINXD_LICENSE_FILE=%LICFILE%")

REM ---- find vivado.bat (first match wins) ----
set "VIVADO="
for %%P in (
    "D:\2026.1\Vivado\bin\vivado.bat"
    "D:\Downloads\Vivado_2026.1\Vivado\bin\vivado.bat"
    "D:\Downloads\Vivado_2026.1\bin\vivado.bat"
    "C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat"
    "C:\Xilinx\Vivado\2026.1\bin\vivado.bat"
) do (
    if not defined VIVADO if exist %%P set "VIVADO=%%~P"
)

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
