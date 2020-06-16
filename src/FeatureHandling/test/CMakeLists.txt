
add_executable(Vocpp_FeatureHandlingTest
    #include
    src/input.h
    #src
    src/OrFastDetectorTest.cpp
    src/OrbTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_FeatureHandlingTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_FeatureHandlingTest
    PRIVATE
        Vocpp_Utils
        Vocpp_FeatureHandling    
        gtest
        ${OpenCV_LIBS} 
)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/input.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/input.h)

add_test(NAME FeatureHandlingTest COMMAND Vocpp_FeatureHandlingTest)