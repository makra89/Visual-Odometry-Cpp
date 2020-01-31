

add_library(Vocpp_FeatureHandling STATIC
    #include
    ./include/Vocpp_FeatureHandling/FeatureDetector.h
    ./include/Vocpp_FeatureHandling/FeatureDescriptor.h
    ./include/Vocpp_FeatureHandling/FeatureMatcher.h
    #src
    ./src/FeatureDetector.cpp
    ./src/FeatureDescriptor.cpp
    ./src/FeatureMatcher.cpp
    ./src/HarrisEdgeDetector.h
    ./src/HarrisEdgeDetector.cpp
    ./src/BriefDescriptor.h
    ./src/BriefDescriptor.cpp
    ./src/BruteForceMatcher.h
    ./src/BruteForceMatcher.cpp
)


target_include_directories(Vocpp_FeatureHandling
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_FeatureHandling
    PRIVATE
        Vocpp_Utils
        ${OpenCV_LIBS} 
)