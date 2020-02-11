
add_executable(Vocpp_DeltaPoseReconstructionTest
    #src
    src/FullFundamentalMat8ptTest.cpp
    src/PureTranslationModelTest.cpp
    src/LocalMapTest.cpp
    src/main.cpp
)

target_include_directories(Vocpp_DeltaPoseReconstructionTest
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        $<TARGET_PROPERTY:Vocpp_DeltaPoseReconstruction,INCLUDE_DIRECTORIES>
)

target_link_libraries(Vocpp_DeltaPoseReconstructionTest
    PRIVATE
        gtest
        ${OpenCV_LIBS} 
        Vocpp_Utils
        Vocpp_DeltaPoseReconstruction
)

add_test(NAME DeltaPoseReconstructionTest COMMAND Vocpp_DeltaPoseReconstructionTest)