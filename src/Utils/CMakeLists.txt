

add_library(Vocpp_Utils STATIC
    #include
    ./include/Vocpp_Utils/ImageProcessingUtils.h
    ./include/Vocpp_Utils/Frame.h
    #src
    ./src/ImageProcessingUtils.cpp
    ./src/Frame.cpp
)


target_include_directories(Vocpp_Utils
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_Utils
    PRIVATE
        ${OpenCV_LIBS} 
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()