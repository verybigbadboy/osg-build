cmake_minimum_required( VERSION 2.8.8 )
project( osgTest )

set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib )
if( WIN32 )
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} )
else()
    set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY} )
endif()

#
# Setup the the lib directory on a per platform basis
#
if(CMAKE_COMPILER_IS_GNUCXX)
    include(GNUInstallDirs)
endif(CMAKE_COMPILER_IS_GNUCXX)

if(NOT DEFINED CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR lib)
endif(NOT DEFINED CMAKE_INSTALL_LIBDIR)

INCLUDE( CMakeMacros )

include( FindOSGHelper )
include( FindBulletHelper )

FIND_PACKAGE( osgWorks 2.0.0 REQUIRED )
if( osgWorks_FOUND )
    include( ${osgWorks_USE_FILE} )
endif()

FIND_PACKAGE( osgBullet 2.0.0 REQUIRED )
if( osgBullet_FOUND )
    include( ${osgBullet_USE_FILE} )
endif()

INCLUDE_DIRECTORIES( 
    ${PROJECT_SOURCE_DIR}/include
    ${OSG_INCLUDE_DIRS}
    #${OSGWORKS_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    ${BULLET_EXTRAS_INCLUDE_DIR}
    ${OSGBULLET_P5_INCLUDE_DIR}
)

# On windows provide the user with the pdb files 
# for debugging if they are present
if(MSVC)
    install(
        DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/\${CMAKE_INSTALL_CONFIG_NAME}/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        USE_SOURCE_PERMISSIONS
        COMPONENT libosgbbullet-dev
        FILES_MATCHING PATTERN "*.pdb"
    )
endif(MSVC)

ADD_EXECUTABLE(test main.cpp)

TARGET_LINK_LIBRARIES(test ${OSGWORKS_LIBRARIES} ${OSG_LIBRARIES} ${BULLET_LIBRARIES}  ${OSGBULLET_LIBRARIES})