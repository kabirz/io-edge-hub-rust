@echo off
rem Flash the full image (bootloader + signed app) at 0x08000000.
setlocal
set STLINK="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"
%STLINK% -P "%~dp0..\build\full.bin" 0x08000000 -Rst
endlocal
