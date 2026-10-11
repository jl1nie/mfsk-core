@echo off
rem Build the C++ driver with MSVC against the MSVC build of mfsk-ffi (#368).
rem
rem   cargo build -p mfsk-ffi --release        (with RUSTFLAGS=-C target-feature=+crt-static
rem                                              for a DLL with no VC++ runtime dependency)
rem   mfsk-ffi\examples\cpp_smoke\build_msvc.bat
rem
rem Writes cpp_smoke.exe and copies mfsk.dll beside it. The driver reads the golden
rem recordings' paths from the environment (build.sh bakes them in instead):
rem
rem   set MFSK_JTTY_WAV=%CD%\embedded-poc\assets\golden\jtty\260807_134110.wav
rem   set MFSK_MSK144_WAV=%CD%\embedded-poc\assets\golden\msk144\181211_120800.wav
rem   mfsk-ffi\examples\cpp_smoke\cpp_smoke.exe
rem
rem `cl` is used if it is already on PATH (a Developer Command Prompt, or CI's
rem msvc-dev-cmd); otherwise the newest Visual Studio with the C++ tools is found
rem with vswhere.
setlocal
where cl >nul 2>&1 && goto build
set "VSW=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSW%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS=%%i"
if not defined VS echo no Visual Studio with the C++ tools found& exit /b 1
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
:build
cd /d "%~dp0"
cl /nologo /utf-8 /std:c++17 /EHsc /O2 /W3 /D_CRT_SECURE_NO_WARNINGS /I ..\..\include main.cpp /Fe:cpp_smoke.exe ..\..\..\target\release\mfsk.dll.lib || exit /b 2
copy /y ..\..\..\target\release\mfsk.dll . >nul || exit /b 3
echo built %CD%\cpp_smoke.exe
