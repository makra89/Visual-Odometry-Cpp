cmake_minimum_required(VERSION 3.12)

# Project name
project(VisualOdometryCpp)

# =========================
# Options
# =========================
option(BUILD_TESTS "Build tests" ON)
option(BUILD_PYTHON_WRAPPER "Generate Python Wrapper" OFF)
option(BUILD_OPENCV "Build opencv as external project" OFF)

# Check for build type, if nothing set, choose Release
if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release )
endif()

if(BUILD_TESTS)
    # Add and configure gtest
    set(gtest_force_shared_crt ON CACHE BOOL "shared run-time lib" FORCE)
    option(BUILD_GMOCK OFF)
    option(INSTALL_GTEST OFF)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/googletest-release-1.10.0)
    enable_testing()
endif()

# Check whether the generator supports the platform identifier
# If yes use it for OpenCV configuration
set(PLATFORM_STRING "")
if(NOT CMAKE_GENERATOR_PLATFORM STREQUAL "")
	set(PLATFORM_STRING "-DCMAKE_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}")
endif()

# Optionally build opencv as external project
# Opencv is build during the CMake configure step
if(BUILD_OPENCV)
    
    set(OpenCV_Install_Dir ${CMAKE_BINARY_DIR}/opencv-install)
    
    # Check for multiconfig generator
    get_property(_isMultiConfig GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)   
    if(NOT ${_isMultiConfig})
        set(OpenCV_LinuxBuildTypeArgs "-DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
    endif()
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/OpenCV-Download/CMakeLists.txt.in
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/OpenCV-Download/CMakeLists.txt)

    execute_process( COMMAND ${CMAKE_COMMAND} ${PLATFORM_STRING} -S ${CMAKE_SOURCE_DIR}/3rdParty/OpenCV-Download -B ${CMAKE_BINARY_DIR}/OpenCV-Download)
    # I did not found another way than always building both Debug and Release for MultiConfig generators
    if(NOT ${_isMultiConfig})
        execute_process( COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/OpenCV-Download )
    else()
        execute_process( COMMAND ${CMAKE_COMMAND} --build . --config Release WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/OpenCV-Download )
        execute_process( COMMAND ${CMAKE_COMMAND} --build . --config Debug WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/OpenCV-Download )
    endif()
    if(WIN32) 
        set(OpenCV_DIR ${OpenCV_Install_Dir} CACHE PATH "OpenCV CMake Package Path" FORCE)
    else() 
        set(OpenCV_DIR ${OpenCV_Install_Dir}/lib/cmake/opencv4 CACHE PATH "OpenCV CMake Package Path" FORCE)
    endif() 
endif()

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add project source
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add applications
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps)

# Generate Python Wrapper
if(BUILD_PYTHON_WRAPPER)

    # Some policies related to SWIG
    cmake_policy(SET CMP0078 NEW)
    cmake_policy(SET CMP0086 NEW)

    include(UseSWIG)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/python)
endif()
