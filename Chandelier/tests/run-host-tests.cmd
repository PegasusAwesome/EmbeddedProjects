@echo off
setlocal
set "taskTestBuild=%TEMP%\chandelier-host-tests"
if not exist "%taskTestBuild%" mkdir "%taskTestBuild%"
for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "taskVsInstall=%%i"
if not defined taskVsInstall exit /b 1
call "%taskVsInstall%\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64 -host_arch=x64 >nul
if errorlevel 1 exit /b 1
pushd "%taskTestBuild%"
cl /nologo /std:c++17 /EHsc /W4 /I"%~dp0stubs" "%~dp0candles.cpp" "%~dp0..\src\CandleEffect.cpp" /Fe:chandelier-host-tests.exe
if errorlevel 1 (popd & exit /b 1)
chandelier-host-tests.exe
set "taskTestResult=%errorlevel%"
popd
exit /b %taskTestResult%
