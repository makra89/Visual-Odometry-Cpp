add_executable( main main.cpp )

target_link_libraries( main 
	${OpenCV_LIBS} 
	Vocpp_FeatureHandling
	Vocpp_Master
	Vocpp_Utils
)

