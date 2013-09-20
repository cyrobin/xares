cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME xares)
set(PACKAGE_VERSION "0.1")

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# Find GDAL ( export GDAL_ROOT=$prefix )
find_package(GDAL REQUIRED)

# Find Boost ( export BOOST_ROOT=$prefix )
set(BOOST_MIN_VERSION "1.46.0") # 1.47 for Boost.Geometry (Ubuntu > 12.04)
# Note: there are some issue with 1.49 (Debian Wheezy); you may use 1.54 from sid
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS graph unit_test_framework python REQUIRED)

## Find Python (optional)
#find_package(PythonLibs)
#find_package(PythonInterp)

include_directories(include)
include_directories(${GDAL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Filesystem Hierarchy Standard
include(GNUInstallDirs)

# Load the 'install_pkg_config_file' macro
include(InstallPkgConfigFile)
find_package(PkgConfig)

# RPath management http://www.cmake.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif("${isSystemDir}" STREQUAL "-1")

# find gladys
pkg_check_modules(PC_LIBGLADYS REQUIRED gladys)
set(LIBGLADYS_DEFINITIONS ${PC_LIBGLADYS_CFLAGS_OTHER})
find_path(LIBGLADYS_INCLUDE_DIR gladys/gladys.hpp
    HINTS ${PC_LIBGLADYS_INCLUDEDIR} ${PC_LIBGLADYS_INCLUDE_DIRS}
    PATH_SUFFIXES gladys)
find_library(LIBGLADYS_LIBRARY NAME gladys
    HINTS ${PC_LIBGLADYS_LIBDIR} ${PC_LIBGLADYS_LIBRARY_DIRS} )
set(LIBGLADYS_INCLUDE_DIRS ${LIBGLADYS_INCLUDE_DIR})
set(LIBGLADYS_LIBRARIES ${LIBGLADYS_LIBRARY})
##

include_directories( ${LIBGLADYS_INCLUDE_DIRS} )
#link_directories( ${LIBGLADYS_LIBRAIRIES} )

# Library and binary
add_subdirectory(src)

## Python bindings
#if (PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
#    include_directories(${PYTHON_INCLUDE_DIRS})
#    add_subdirectory(python)
#else()
#    message("Hey! Install Python [if you wanna Xaresplay]")
#endif(PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)

# Testing
include( CTest )
add_subdirectory(test)

# Install headers
file(GLOB xares_HDRS "include/xares/*.hpp")
install(FILES ${xares_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/xares)

#########################################################################
### Uninstall ###
#########################################################################
configure_file("${CMAKE_HOME_DIRECTORY}/CMakeModules/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")
