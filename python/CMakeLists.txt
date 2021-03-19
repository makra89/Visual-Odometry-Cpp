cmake_policy(SET CMP0078 NEW)

find_package(SWIG REQUIRED)

set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)

# Include python
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(Python COMPONENTS NumPy)
include_directories(${Python_NumPy_INCLUDE_DIRS})

set_property(SOURCE vocpp.i PROPERTY CPLUSPLUS ON)

swig_add_library(vocpp_python
     LANGUAGE python 
     SOURCES vocpp.i
)

set_property(TARGET vocpp_python PROPERTY SWIG_INCLUDE_DIRECTORIES 
    $<TARGET_PROPERTY:Vocpp_Interface,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:Vocpp_Master,INCLUDE_DIRECTORIES>
)

target_link_libraries(vocpp_python
    ${PYTHON_LIBRARIES}
    Vocpp_Master
    Vocpp_DeltaPoseReconstruction
    Vocpp_FeatureHandling
    Vocpp_Calibration
    Vocpp_Utils
    ${OpenCV_LIBS} 
)

add_custom_command(
    TARGET vocpp_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/vocpp
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:vocpp_python>
        ${CMAKE_CURRENT_BINARY_DIR}/vocpp/$<TARGET_FILE_NAME:vocpp_python>
    COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/vocpp.py
        ${CMAKE_CURRENT_BINARY_DIR}/vocpp/vocpp.py      
)

# Files to install with Python
set(VOCPP_PACKAGE_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/vocpp
)

# Configure setup.py and copy to output directory
set(SETUP_PY_IN ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in)
set(SETUP_PY_OUT ${CMAKE_CURRENT_BINARY_DIR}/setup.py)
configure_file(${SETUP_PY_IN} ${SETUP_PY_OUT})

# Install target to call setup.py
add_custom_target(vocpp_python_install
        DEPENDS vocpp_python
        COMMAND python ${SETUP_PY_OUT} install)