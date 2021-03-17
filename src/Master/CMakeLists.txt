

add_library(Vocpp_Master STATIC
    #include
    ./include/Vocpp_Master/Master.h
    #src
    ./src/Master.cpp
)


target_include_directories(Vocpp_Master
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_Master
    PUBLIC
        Vocpp_Interface
    PRIVATE
        Vocpp_DeltaPoseReconstruction
        Vocpp_FeatureHandling
        Vocpp_Calibration
        Vocpp_Utils
        ${OpenCV_LIBS} 
)