
add_library(Vocpp_Interface INTERFACE)

target_include_directories(Vocpp_Interface
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Vocpp_Interface
    INTERFACE
        ${OpenCV_LIBS} 
)

if(BUILD_TESTS)
    add_subdirectory(test)
endif()