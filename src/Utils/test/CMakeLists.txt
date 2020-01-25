
add_executable(Vocpp_UtilsTest
    #src
    src/FrameTest.cpp
    src/ImageProcessingUtilsTest.cpp
    src/NumericalUtilitiesTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_UtilsTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_UtilsTest
    PRIVATE
        Vocpp_Utils    
        gtest
)

add_test(NAME UtilsTest COMMAND Vocpp_UtilsTest)