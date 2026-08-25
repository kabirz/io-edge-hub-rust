@echo off
REM 一键 MSVC 构建脚本: 定位 Visual Studio -> vcvars64 -> cmake 配置 + 编译
REM 用法: tools\build.bat
REM 退出码: 0=成功 1=未找到 VS 2=vcvars64 失败 3=配置失败 4=编译失败

setlocal

rem 定位 Visual Studio (vswhere 展开 %ProgramFiles(x86)% 较为棘手, 先存入变量)
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.Component.MSBuild -property installationPath`) do set VS=%%i
if not defined VS (
    echo [错误] 未找到 Visual Studio
    exit /b 1
)

call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
if errorlevel 1 (
    echo [错误] vcvars64 失败
    exit /b 2
)

rem 切到项目根 (脚本位于 tools/ 下)
cd /d "%~dp0\.."

cmake --preset vs
if errorlevel 1 (
    echo [错误] configure 失败
    exit /b 3
)

cmake --build out --config Release
if errorlevel 1 (
    echo [错误] build 失败
    exit /b 4
)

echo [成功] 构建完成: out\bin\Release\io-edge-hub-host.exe
exit /b 0
