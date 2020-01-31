

add_library(Vocpp_DeltaPoseReconstruction STATIC
    #include
    ./include/Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h
    #src
    ./src/DeltaPoseReconstructor.cpp
    ./src/EpipolarModel.h
    ./src/RansacOptimizer.h
    ./src/RansacOptimizer.cpp
    ./src/FullFundamentalMat8pt.h
    ./src/FullFundamentalMat8pt.cpp
)


target_include_directories(Vocpp_DeltaPoseReconstruction
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_DeltaPoseReconstruction
    PRIVATE
        ${OpenCV_LIBS} 
        Vocpp_FeatureHandling
        Vocpp_Utils
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()