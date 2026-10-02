@echo off
setlocal EnableDelayedExpansion
title Vivado build - freq-lab FPGA (lockin, photon_counter, correlator, vlf_sid)

REM ================================================================
REM  Vivado batch build - PZ7020-StarLite (xc7z020clg400-2)
REM  Builds lockin, photon_counter, correlator, vlf_sid one after another.
REM  Progress: build_status.txt    Vivado logs: log_*.log  (this folder)
REM ================================================================

set "FPGA_DIR=%~dp0"
if "%FPGA_DIR:~-1%"=="\" set "FPGA_DIR=%FPGA_DIR:~0,-1%"
set "STATUS=%FPGA_DIR%\build_status.txt"

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

REM ---- 1/4 lockin ----
echo.
echo [1/4] BUILDING LOCKIN ...
echo lockin: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\lockin"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_lockin.log" -journal "%FPGA_DIR%\jou_lockin.jou"
set "EC=!errorlevel!"
echo lockin: EXIT_CODE=!EC! at %time%>> "%STATUS%"

REM ---- 2/4 photon_counter ----
echo.
echo [2/4] BUILDING PHOTON_COUNTER ...
echo photon_counter: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\photon_counter"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_photon.log" -journal "%FPGA_DIR%\jou_photon.jou"
set "EC=!errorlevel!"
echo photon_counter: EXIT_CODE=!EC! at %time%>> "%STATUS%"

REM ---- 3/4 correlator ----
echo.
echo [3/4] BUILDING CORRELATOR ...
echo correlator: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\correlator"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_correlator.log" -journal "%FPGA_DIR%\jou_correlator.jou"
set "EC=!errorlevel!"
echo correlator: EXIT_CODE=!EC! at %time%>> "%STATUS%"

REM ---- 4/4 vlf_sid ----
echo.
echo [4/4] BUILDING VLF_SID ...
echo vlf_sid: STARTED %time%>> "%STATUS%"
cd /d "%FPGA_DIR%\vlf_sid"
call "%VIVADO%" -mode batch -source build.tcl -log "%FPGA_DIR%\log_vlf_sid.log" -journal "%FPGA_DIR%\jou_vlf_sid.jou"
set "EC=!errorlevel!"
echo vlf_sid: EXIT_CODE=!EC! at %time%>> "%STATUS%"

echo COMPLETED: %date% %time%>> "%STATUS%"
echo.
echo ================================================================
echo  ALL BUILDS FINISHED - results in build_status.txt
echo  (this window can be closed)
echo ================================================================
pause
