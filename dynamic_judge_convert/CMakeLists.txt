add_executable( dynamic_judge_convert dynamic_judge_convert.cpp )
target_link_libraries( dynamic_judge_convert myslam )
target_link_libraries( dynamic_judge_convert ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} libOpenNI2.so )
