add_executable( check_imu_mask check_imu_mask.cpp )
target_link_libraries( check_imu_mask myslam )
target_link_libraries( check_imu_mask ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} libOpenNI2.so )
