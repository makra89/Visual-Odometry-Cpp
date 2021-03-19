

add_library(Vocpp_DeltaPoseReconstruction STATIC
    #include
    ./include/Vocpp_DeltaPoseReconstruction/DeltaPoseReconstructor.h
    ./include/Vocpp_DeltaPoseReconstruction/LocalMap.h
    #src
    ./src/DeltaPoseReconstructor.cpp
    ./src/EpipolarSolution.h
    ./src/EpipolarSolution.cpp
    ./src/LocalMap.cpp
)


target_include_directories(Vocpp_DeltaPoseReconstruction
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_DeltaPoseReconstruction
    PUBLIC
        Vocpp_Interface
        Vocpp_FeatureHandling
        Vocpp_Utils
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()
