

add_library(Vocpp_Master STATIC
    #include
    ./include/Vocpp_Master/VocppMaster.h
    #src
    ./src/VocppMaster.cpp
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
        Vocpp_DeltaPoseReconstruction
        Vocpp_Calibration
    PRIVATE
        Vocpp_FeatureHandling
        Vocpp_Utils
)