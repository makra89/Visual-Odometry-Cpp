

add_library(Vocpp_FeatureHandling STATIC
    #include
    ./include/Vocpp_FeatureHandling/Common.h
    ./include/Vocpp_FeatureHandling/BriefDescriptor.h
    ./include/Vocpp_FeatureHandling/HarrisEdgeDetector.h
    ./include/Vocpp_FeatureHandling/BruteForceBinaryMatcher.h
    ./include/Vocpp_FeatureHandling/OrbDetectorDescriptor.h
    ./include/Vocpp_FeatureHandling/OrFastDetector.h
    #src   
    ./src/HarrisEdgeDetector.cpp
    ./src/BriefDescriptor.cpp
    ./src/BruteForceBinaryMatcher.cpp
    ./src/OrFastDetector.cpp
)


target_include_directories(Vocpp_FeatureHandling
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_FeatureHandling
    PUBLIC
        Vocpp_Interface
    PRIVATE
        Vocpp_Utils
        ${OpenCV_LIBS} 
)