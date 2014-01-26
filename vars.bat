echo on
call "%VS90COMNTOOLS%\..\..\VC\vcvarsall.bat"

SET BUILD_ROOT_DIR=%~dp0
SET PATH=%PATH%;%BUILD_ROOT_DIR%\osg
SET PATH=%PATH%;%BUILD_ROOT_DIR%\bullet
SET PATH=%PATH%;%BUILD_ROOT_DIR%\bullet\src
SET PATH=%PATH%;%BUILD_ROOT_DIR%\osgWorks
SET PATH=%PATH%;%BUILD_ROOT_DIR%\osgBullet
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osg
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osg\bin
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\bin
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\lib\Release
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\lib\Debug
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\bullet\src
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgWorks\lib
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgWorks\bin\Debug
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgWorks\bin\Release
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgBullet\lib
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgBullet\bin\Debug
SET PATH=%PATH%;%BUILD_ROOT_DIR%\build\osgBullet\bin\Release

SET OSG_FILE_PATH=%BUILD_ROOT_DIR%\osgBullet\data
