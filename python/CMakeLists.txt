cmake_policy(SET CMP0078 NEW)

find_package(SWIG REQUIRED)

set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)

# Include python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

set_property(SOURCE Vocpp.i PROPERTY CPLUSPLUS ON)

swig_add_library(Vocpp_python
     LANGUAGE python 
     SOURCES Vocpp.i
)

set_property(TARGET Vocpp_python PROPERTY SWIG_INCLUDE_DIRECTORIES 
    $<TARGET_PROPERTY:Vocpp_Interface,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Vocpp_Master,INCLUDE_DIRECTORIES>
)

target_link_libraries(Vocpp_python
    ${PYTHON_LIBRARIES}
    Vocpp_Master
    Vocpp_DeltaPoseReconstruction
    Vocpp_FeatureHandling
    Vocpp_Calibration
    Vocpp_Utils
    ${OpenCV_LIBS} 
)

add_custom_command(
    TARGET Vocpp_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/Vocpp
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:Vocpp_python>
        ${CMAKE_CURRENT_BINARY_DIR}/Vocpp/$<TARGET_FILE_NAME:Vocpp_python>
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/Vocpp.py
        ${CMAKE_CURRENT_BINARY_DIR}/Vocpp/Vocpp.py      
)

# Files to install with Python
set(VOCPP_PACKAGE_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/Vocpp
)

# Configure setup.py and copy to output directory
set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in)
set(SETUP_PY_OUT ${CMAKE_CURRENT_BINARY_DIR}/setup.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

# Install target to call setup.py
add_custom_target(Vocpp_python_install
        DEPENDS Vocpp_python
        COMMAND python ${SETUP_PY_OUT} install)