
add_executable(Vocpp_MasterTest
    #include
    #src
    src/MasterTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_MasterTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Vocpp_MasterTest
    PRIVATE
        Vocpp_Master  
        gtest
)

add_test(NAME MasterTest COMMAND Vocpp_MasterTest)