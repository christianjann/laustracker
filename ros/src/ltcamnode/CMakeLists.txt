cmake_minimum_required(VERSION 2.8)
project(ltcamnode)

set(LOCAL_UEYE_CAM_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libueyecam)
set(LOCAL_LAUSTRACKER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/share/cmake)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg cv_bridge
                                        image_transport  message_generation)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES setAutoExposure.srv)
add_service_files(FILES setAutoGain.srv)
add_service_files(FILES setExposureTime.srv)
add_service_files(FILES setFrameRate.srv)
add_service_files(FILES setHardwareGain.srv)
add_service_files(FILES getMaxExposureTime.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

#catkin_package(LIBRARIES ${PROJECT_NAME})

#SET(CMAKE_VERBOSE_MAKEFILE ON)
#SET(CMAKE_BUILD_TYPE debug)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0"
"1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0"
"1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0")
find_package(Boost 1.37 COMPONENTS "date_time" "filesystem" "graph"
              "iostreams" "program_options" "regex" "serialization" "signals"
              "system" "thread" "wserialization" REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

#find_package(UEye REQUIRED)
find_package(UEyeCam REQUIRED)
find_package(OpenCV REQUIRED )
find_package(LausTracker REQUIRED )

set(INCLUDE_DIRS ${INCLUDE_DIRS} include)
set(INCLUDE_DIRS ${INCLUDE_DIRS} ${UEye_INCLUDE_DIR} ${UEyeCam_INCLUDE_DIR}
                 ${LausTracker_INCLUDE_DIR})
include_directories(${INCLUDE_DIRS})
add_executable(laustracker-camnode laustracker-camnode.cpp)
target_link_libraries(laustracker-camnode ${UEye_LIBRARIES} ${UEyeCam_LIBRARIES}
                                          ${OpenCV_LIBS} ${LausTracker_LIBRARIES}
                                          ${catkin_LIBRARIES}
                                          ${Boost_LIBRARIES})
add_dependencies(laustracker-camnode ltcamnode_gencpp)
