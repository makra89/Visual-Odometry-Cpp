add_executable( TrackFeatures TrackFeatures.cpp )

target_link_libraries( TrackFeatures 
	${OpenCV_LIBS} 
	Vocpp_FeatureHandling
        Vocpp_DeltaPoseReconstruction
	Vocpp_Master
	Vocpp_Utils
)

