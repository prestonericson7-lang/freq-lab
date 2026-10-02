@echo off
setlocal EnableDelayedExpansion
title Vivado build - freq-lab ring-oscillator TRNG (#73)

set "FPGA_DIR=P:\Downloads\freq-lab\fpga"
set "STATUS=%FPGA_DIR%\build_status_trng.txt"
set "LICFILE=P:\Downloads\Xilinx.lic"
if defined XILINXD_LICENSE_FILE (set "XILINXD_LICENSE_FILE=%LICFILE%;%XILINXD_LICENSE_FILE%") else (set "XILINXD_LICENSE_FILE=%LICFILE%")

set "VIVADO="
for %%P in (
    "D:\2026.1\Vivado\bin\vivado.bat"
    "D:\Downloads\Vivado_2026.1\Vivado\bin\vivado.bat"
    "C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat"
    "C:\Xilinx\Vivado\2026.1\bin\vivado.bat"
) do ( if not defined VIVADO if exist %%P set "VIVADO=%%~P" )
if not defined VIVADO ( echo ERROR: vivado.bat not found> "%STATUS%" & pause & exit /b 1 )

echo Vivado: %VIVADO%> "%STATUS%"
echo License: %XILINXD_LICENSE_FILE%>> "%STATUS%"
echo Started: %date% %time%>> "%STATUS%"
echo trng: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\trng"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_trng.log" -journal "%FPGA_DIR%\jou_trng.jou"
set "EC=!errorlevel!"
echo trng: EXIT_CODE=!EC! at %time%>> "%STATUS%"
echo COMPLETED: %date% %time%>> "%STATUS%"
echo.
echo  TRNG BUILD FINISHED (exit code !EC!) - see build_status_trng.txt
echo  Bitstream: %FPGA_DIR%\trng\build\trng.bit
if "%1"=="" pause
