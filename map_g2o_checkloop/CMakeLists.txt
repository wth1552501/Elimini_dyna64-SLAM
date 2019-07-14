add_executable( map_g2o_checkloop map_g2o_checkloop.cpp )
target_link_libraries( map_g2o_checkloop myslam )
target_link_libraries( map_g2o_checkloop ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${THIRD_PARTY_LIBS} libOpenNI2.so)
