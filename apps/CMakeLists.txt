add_executable( runKittiDemo runKittiDemo.cpp )

target_link_libraries( runKittiDemo
    ${OpenCV_LIBS} 
    Vocpp_FeatureHandling
    Vocpp_DeltaPoseReconstruction
    Vocpp_Calibration
    Vocpp_Master
    Vocpp_Utils
)

