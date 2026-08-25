@echo off
rem Flash the raw app to the active partition (0x08020000); keeps the
rem on-device embassy-boot bootloader untouched.
setlocal
set STLINK="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
%STLINK% -P "%~dp0..\build\app.bin" 0x08020000 -Rst
endlocal
