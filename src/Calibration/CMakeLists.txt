

add_library(Vocpp_Calibration STATIC
    #include
    ./include/Vocpp_Calibration/CalibrationModule.h
    ./include/Vocpp_Calibration/MonoCameraCalibration.h
    #src
    ./src/CalibrationModule.cpp
)


target_include_directories(Vocpp_Calibration
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_Calibration
    PUBLIC
        Vocpp_Interface
    PRIVATE
        Vocpp_Utils
)