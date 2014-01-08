echo on
call "%VS90COMNTOOLS%\..\..\VC\vcvarsall.bat"

SET BUILD_ROOT_DIR=%~dp0

SET PATH=%PATH%;%BUILD_ROOT_DIR%\osg
SET PATH=%PATH%;%BUILD_ROOT_DIR%\bullet
SET PATH=%PATH%;%BUILD_ROOT_DIR%\bullet\src
SET PATH=%PATH%;%BUILD_ROOT_DIR%\osgWorks
SET PATH=%PATH%;%BUILD_ROOT_DIR%\osgBullet

mkdir build
mkdir build\osg
mkdir build\bullet
mkdir build\osgWorks
mkdir build\osgBullet

cd %BUILD_ROOT_DIR%
cd build\osg
cmake ..\..\osg

cd %BUILD_ROOT_DIR%
devenv build\osg\OpenSceneGraph.sln /Build "Debug|Win32"
devenv build\osg\OpenSceneGraph.sln /Build "Release|Win32"

SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osg
xcopy build\osg\include osg\include /E /y

cd %BUILD_ROOT_DIR%
cd build\bullet
cmake ..\..\bullet

cd %BUILD_ROOT_DIR%
devenv build\bullet\BULLET_PHYSICS.sln /Build "Debug|Win32"
devenv build\bullet\BULLET_PHYSICS.sln /Build "Release|Win32"

SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\lib\Release
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\lib\Debug
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\src

cd %BUILD_ROOT_DIR%
cd build\osgWorks
cmake ..\..\osgWorks

cd %BUILD_ROOT_DIR%
devenv build\osgWorks\osgWorks.sln /Build "Debug|Win32"
devenv build\osgWorks\osgWorks.sln /Build "Release|Win32"

SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgWorks\lib

cd %BUILD_ROOT_DIR%
cd build\osgBullet
cmake ..\..\osgBullet

cd %BUILD_ROOT_DIR%
devenv build\osgBullet\osgBullet.sln /Build "Debug|Win32"
devenv build\osgBullet\osgBullet.sln /Build "Release|Win32"
