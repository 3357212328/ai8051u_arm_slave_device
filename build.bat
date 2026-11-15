@echo off
setlocal EnableDelayedExpansion

title Keil Auto Rebuild

echo ======================================
echo       Keil Auto Rebuild Script
echo ======================================
echo.

cd /d "%~dp0"

::------------------------------------------
:: find project file
::------------------------------------------
set PROJECT=

for %%i in (*.uvprojx) do (
    set PROJECT=%%i
    goto FoundProject
)

for %%i in (*.uvproj) do (
    set PROJECT=%%i
    goto FoundProject
)

echo [ERROR] No Keil project found in current directory.
echo.
pause
exit /b 1

:FoundProject

echo Project:
echo    %PROJECT%
echo.

::------------------------------------------
:: Keil path
::------------------------------------------
set UV4=D:\Keil_v5\UV4\UV4.exe

if not exist "%UV4%" (
    echo [ERROR] Keil executable not found:
    echo    %UV4%
    echo.
    pause
    exit /b 1
)

echo Keil:
echo    %UV4%
echo.

echo Rebuilding project...
echo.

::------------------------------------------
:: Rebuild All
::------------------------------------------
"%UV4%" -r "%PROJECT%" -j0 -o build.log

echo.
echo ======================================

findstr /C:"0 Error(s)" build.log >nul

if %errorlevel%==0 (
    echo Rebuild Succeeded!
) else (
    echo Rebuild Failed!
)

echo ======================================
echo.
echo Build Log:
echo    %cd%\build.log
echo.
pause
