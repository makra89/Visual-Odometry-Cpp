
add_executable(Vocpp_UtilsTest
    #src
    src/ImageProcessingUtilsTest.cpp
    src/NumericalUtilitiesTest.cpp
    src/FrameRotationsTest.cpp
    src/IntImageTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_UtilsTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_UtilsTest
    PRIVATE
        Vocpp_Utils    
        Vocpp_Interface    
        gtest
)

add_test(NAME UtilsTest COMMAND Vocpp_UtilsTest)