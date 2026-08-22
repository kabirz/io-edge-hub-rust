@echo off
rem Flash the signed app to slot0 (0x08010000); keeps the on-device MCUboot untouched.
setlocal
set STLINK="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
%STLINK% -P "%~dp0..\build\app.signed.bin" 0x08010000 -Rst
endlocal
