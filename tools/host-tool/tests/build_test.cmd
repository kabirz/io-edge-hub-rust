@echo off
rem Build tests\protocol_test.exe against the tool's own protocol sources
setlocal
cd /d "%~dp0.."
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.Component.MSBuild -property installationPath`) do set VS=%%i
if not defined VS (
    echo [error] Visual Studio not found
    exit /b 1
)
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
cl /utf-8 /O2 /W4 /D_CRT_SECURE_NO_WARNINGS /Iinclude tests\protocol_test.c src\udp_manager.c src\fw_image.c ^
   /link ws2_32.lib iphlpapi.lib /out:out\protocol_test.exe
if errorlevel 1 exit /b 1
del /q tests\protocol_test.obj src\udp_manager.obj src\fw_image.obj 2>nul
echo [ok] out\protocol_test.exe
