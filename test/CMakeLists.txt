add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo myslam )
target_link_libraries( run_vo ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} libOpenNI2.so )
