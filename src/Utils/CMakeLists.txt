

add_library(Vocpp_Utils STATIC
    #include
    ./include/Vocpp_Utils/ImageProcessingUtils.h
    ./include/Vocpp_Utils/NumericalUtilities.h
    ./include/Vocpp_Utils/ConversionUtils.h
    ./include/Vocpp_Utils/FrameRotations.h
    ./include/Vocpp_Utils/IntImage.h
    ./include/Vocpp_Utils/TracingImpl.h
    #src
    ./src/ImageProcessingUtils.cpp
    ./src/NumericalUtilities.cpp
    ./src/ConversionUtils.cpp
    ./src/FrameRotations.cpp
    ./src/IntImage.cpp
    ./src/TracingImpl.cpp
)


target_include_directories(Vocpp_Utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_Utils
    PUBLIC
        Vocpp_Interface
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()