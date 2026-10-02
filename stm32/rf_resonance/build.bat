@echo off
REM Compile rf_resonance with arduino-cli for both STM32 targets.
REM STM32 core 3.x takes typed pin modes (PinMode / PinStatus); RadioLib 7.x leaves the
REM casts empty for that core, so they are passed here. Nothing in the library is edited.
REM Upload: add  --upload -p COMx  (BlackPill F411CE over DFU, or an ST-Link for the BluePill).
set FLAGS=compiler.cpp.extra_flags=-DRADIOLIB_ARDUINOHAL_PIN_MODE_CAST=(PinMode) -DRADIOLIB_ARDUINOHAL_PIN_STATUS_CAST=(PinStatus) -DRADIOLIB_ARDUINOHAL_INTERRUPT_MODE_CAST=(PinStatus)
cd /d "%~dp0"
echo === BluePill F103C8 ===
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8 --build-property "%FLAGS%" .
echo === BlackPill F411CE ===
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF4:pnum=BLACKPILL_F411CE --build-property "%FLAGS%" .
if "%1"=="" pause
