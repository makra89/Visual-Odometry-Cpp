cmake_minimum_required(VERSION 3.12)

# Project name
project(VoForDummies)

# Choose whether to build tests
option(BUILD_TESTS "Build tests" ON)

if(BUILD_TESTS)
    # Add and configure gtest
    set(gtest_force_shared_crt ON CACHE BOOL "shared run-time lib" FORCE)
    option(BUILD_GMOCK OFF)
    option(INSTALL_GTEST OFF)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/googletest-release-1.10.0)
    enable_testing()
endif()

# Find opencv package
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add project source
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Add applications
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/apps)