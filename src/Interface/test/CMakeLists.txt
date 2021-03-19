
add_executable(Vocpp_InterfaceTest
    #src
    src/FrameTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_InterfaceTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_InterfaceTest
    PRIVATE
        Vocpp_Interface  
        gtest
)

add_test(NAME InterfaceTest COMMAND Vocpp_InterfaceTest)