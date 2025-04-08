@echo off

set "PHYSICS_ROOT=%~dp0"

set "INSTALL_RELEASE=%PHYSICS_ROOT%install-release"
set "INSTALL_DEBUG=%PHYSICS_ROOT%install-debug"
set "BUILD_RELEASE=%PHYSICS_ROOT%build-release"
set "BUILD_DEBUG=%PHYSICS_ROOT%build-debug"

:: Remove old build and install folders
if exist "%INSTALL_RELEASE%" rmdir /S /Q "%INSTALL_RELEASE%"
if exist "%INSTALL_DEBUG%" rmdir /S /Q "%INSTALL_DEBUG%"
if exist "%BUILD_RELEASE%" rmdir /S /Q "%BUILD_RELEASE%"
if exist "%BUILD_DEBUG%" rmdir /S /Q "%BUILD_DEBUG%"

:: Build and install
cmd /c "build_windows_release.bat"
cmd /c "build_windows_debug.bat"

set "ORIGIN_RELEASE_DLL=%INSTALL_RELEASE%\bin\KalaPhysics.dll"
set "ORIGIN_RELEASE_LIB=%INSTALL_RELEASE%\lib\KalaPhysics.lib"
set "ORIGIN_DEBUG_DLL=%INSTALL_DEBUG%\bin\KalaPhysicsD.dll"
set "ORIGIN_DEBUG_LIB=%INSTALL_DEBUG%\lib\KalaPhysicsD.lib"
set "ORIGIN_HEADER1=%PHYSICS_ROOT%\install-release\include\collider.hpp"
set "ORIGIN_HEADER2=%PHYSICS_ROOT%\install-release\include\collisiondetection.hpp"
set "ORIGIN_HEADER3=%PHYSICS_ROOT%\install-release\include\gameobjecthandle.hpp"
set "ORIGIN_HEADER4=%PHYSICS_ROOT%\install-release\include\physicsworld.hpp"
set "ORIGIN_HEADER5=%PHYSICS_ROOT%\install-release\include\rigidbody.hpp"

if not exist "%ORIGIN_RELEASE_DLL%" (
	echo Failed to find origin release dll from '%ORIGIN_RELEASE_DLL%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_RELEASE_LIB%" (
	echo Failed to find origin release lib from '%ORIGIN_RELEASE_LIB%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_DEBUG_DLL%" (
	echo Failed to find origin debug dll from '%ORIGIN_DEBUG_DLL%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_DEBUG_LIB%" (
	echo Failed to find origin debug lib from '%ORIGIN_DEBUG_LIB%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_HEADER1%" (
	echo Failed to find origin header from '%ORIGIN_HEADER1%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_HEADER2%" (
	echo Failed to find origin header from '%ORIGIN_HEADER2%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_HEADER3%" (
	echo Failed to find origin header from '%ORIGIN_HEADER3%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_HEADER4%" (
	echo Failed to find origin header from '%ORIGIN_HEADER4%'!
	pause
	exit /b 1
)
if not exist "%ORIGIN_HEADER5%" (
	echo Failed to find origin header from '%ORIGIN_HEADER5%'!
	pause
	exit /b 1
)

set "TARGET_ROOT=%PHYSICS_ROOT%..\Elypso-engine\_external_shared\KalaPhysics"

if not exist "%TARGET_ROOT%" (
	echo Failed to find target root from '%TARGET_ROOT%'!
	pause
	exit /b 1
)

set "TARGET_RELEASE_DLL=%TARGET_ROOT%\release\KalaPhysics.dll"
set "TARGET_RELEASE_LIB=%TARGET_ROOT%\release\KalaPhysics.lib"
set "TARGET_DEBUG_DLL=%TARGET_ROOT%\debug\KalaPhysicsD.dll"
set "TARGET_DEBUG_LIB=%TARGET_ROOT%\debug\KalaPhysicsD.lib"
set "TARGET_HEADER1=%TARGET_ROOT%\collider.hpp"
set "TARGET_HEADER2=%TARGET_ROOT%\collisiondetection.hpp"
set "TARGET_HEADER3=%TARGET_ROOT%\gameobjecthandle.hpp"
set "TARGET_HEADER4=%TARGET_ROOT%\physicsworld.hpp"
set "TARGET_HEADER5=%TARGET_ROOT%\rigidbody.hpp"

:: Create release and debug folders in case they dont exist yet
if not exist "%TARGET_ROOT%\release" mkdir "%TARGET_ROOT%\release"
if not exist "%TARGET_ROOT%\debug" mkdir "%TARGET_ROOT%\debug"

:: Copy dll files, lib files and header file to target path
copy /Y "%ORIGIN_RELEASE_DLL%" "%TARGET_RELEASE_DLL%"
copy /Y "%ORIGIN_RELEASE_LIB%" "%TARGET_RELEASE_LIB%"
copy /Y "%ORIGIN_DEBUG_DLL%" "%TARGET_DEBUG_DLL%"
copy /Y "%ORIGIN_DEBUG_LIB%" "%TARGET_DEBUG_LIB%"
copy /Y "%ORIGIN_HEADER1%" "%TARGET_HEADER1%"
copy /Y "%ORIGIN_HEADER2%" "%TARGET_HEADER2%"
copy /Y "%ORIGIN_HEADER3%" "%TARGET_HEADER3%"
copy /Y "%ORIGIN_HEADER4%" "%TARGET_HEADER4%"
copy /Y "%ORIGIN_HEADER5%" "%TARGET_HEADER5%"

echo Successfully installed KalaPhysics!

pause
exit /b 0