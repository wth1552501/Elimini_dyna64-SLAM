cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
 
SET(OpenNI2_INCLUDE_DIR /home/wth/dependency/kinectdrive/OpenNI-Linux-x64-2.2/Include )
SET(OpenNI2_LIBRARYS  /home/wth/dependency/kinectdrive/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so)

############### dependencies ######################
#Python 3.5
#include_directories("/usr/local/lib/python3.5/dist-packages")

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.4.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

# pcl 
find_package( PCL 1.9 REQUIRED )
include_directories( ${PCL_INCLUDE_DIRS}
                     /usr/local/include/pcl-1.9)
add_definitions( ${PCL_DEFINITIONS} )


# octomap 
#find_package( octomap REQUIRED )
#include_directories( ${OCTOMAP_INCLUDE_DIRS} )


#Openni2
FIND_PATH(OpenNI2_INCLUDE_DIR OpenNI.h HINTS $ENV{OPENNI2_INCLUDE64} $ENV{OPENNI2_INCLUDE})
FIND_LIBRARY(OpenNI2_LIBRARYS NAMES OpenNI2 HINTS $ENV{OPENNI2_LIB64} $ENV{OPENNI2_LIB} $ENV{OPENNI2_REDIST})
IF (OpenNI2_INCLUDE_DIR AND OpenNI2_LIBRARY)
  SET(OpenNI2_FOUND TRUE)
ENDIF (OpenNI2_INCLUDE_DIR AND OpenNI2_LIBRARY)



IF (OpenNI2_FOUND)
# show which OpenNI2 was found only if not quiet
SET(OpenNI2_LIBRARIES ${OpenNI2_LIBRARYS})

	MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_INCLUDE_DIR}")
	MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_LIBRARYS}")

ELSE (OpenNI2_FOUND)
# fatal error if OpenNI2 is required but not found
IF (OpenNI2_FIND_REQUIRED)
	MESSAGE(FATAL_ERROR "Could not find OpenNI2. Environment variables OPENNI2_INCLUDE (directory containing OpenNI.h) and OPENNI2_LIB (directory containing OpenNI2 library) could bet set.")
ENDIF (OpenNI2_FIND_REQUIRED)
ENDIF (OpenNI2_FOUND) 



include_directories( ${OpenNI2_INCLUDE_DIR} )
link_directories($ENV{OPENNI2_REDIST})


set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d 
)





include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory(src)
#add_subdirectory(test)
add_subdirectory(dynamic_judge_convert)
#add_subdirectory(whole_system)
add_subdirectory(map_g2o_checkloop)
add_subdirectory(check_imu_mask)
#add_subdirectory(ui)



# set( CMAKE_BUILD_TYPE "Debug" )




