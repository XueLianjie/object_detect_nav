#this is for kinectV1 using OpenNI
cmake_minimum_required(VERSION 2.8)
project(Kinect1_reader)
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#set(CMAKE_BUILD_TYPE "Debug")
set( CMAKE_BUILD_TYPE Release )
#Eigen
include_directories("/usr/include/eigen3/")

#OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#PCL
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


#OpenNI  
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)    
find_package(OpenNI REQUIRED)  
#IF(OPENNI_INCLUDE_DIR AND OPENNI_LIBRARY_DIR)    
MESSAGE(STATUS"FoundOpenNIlib")    
INCLUDE_DIRECTORIES(${OPENNI_INCLUDE_DIR}  )    
MESSAGE(STATUS"includediris" ${OPENNI_INCLUDE_DIR})    
MESSAGE(STATUS"libdiris" ${OPENNI_LIBRARY_DIR})    

add_library(kinectreader kinectreader.cpp)
#add_library(config config.cpp)
ADD_EXECUTABLE(readKinect test.cpp )#readKinect.cpp) 


TARGET_LINK_LIBRARIES(readKinect  
#  config
  kinectreader
  ${OpenCV_LIBS}    
  ${OPENNI_LIBRARY_DIR}
  ${PCL_LIBRARIES}
  )  
