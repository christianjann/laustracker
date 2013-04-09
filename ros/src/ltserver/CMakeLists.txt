cmake_minimum_required(VERSION 2.8)
project(ltserver)

set(LOCAL_UEYE_CAM_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../../thirdparty/libueyecam)
set(LOCAL_LAUSTRACKER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/share/cmake)

set(CMAKE_CXX_FLAGS "-std=c++0x")

include(ecl_platform_detection)
ecl_detect_distro()
if(${DISTRO_NAME} MATCHES "Ubuntu")
message("-- Building Laustracker Server on Ubuntu ${DISTRO_VERSION}
        ${DISTRO_VERSION_STRING}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUBUNTU")
endif(${DISTRO_NAME} MATCHES "Ubuntu")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg cv_bridge
             message_generation image_transport ltcamnode)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(FILES Pos.msg)
add_message_files(FILES Idx.msg)
add_message_files(FILES Robot.msg)
add_message_files(FILES Robots.msg)
add_message_files(FILES Field.msg)
add_message_files(FILES Map.msg)

generate_messages(DEPENDENCIES std_msgs ltcamnode)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime std_msgs ltcamnode)

include_directories(${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
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


find_package(UEyeCam REQUIRED )
include_directories(${UEyeCam_INCLUDE_DIR})
find_package(OpenCV REQUIRED )
find_package(LausTracker REQUIRED )
include_directories(${LausTracker_INCLUDE_DIR})

add_executable(laustrackerd server.cpp)
target_link_libraries(laustrackerd ${Boost_LIBRARIES} ${LausTracker_LIBRARIES}
                                   ${OpenCV_LIBS} ${UEyeCam_LIBRARIES}
                                   ${catkin_LIBRARIES})
add_dependencies(laustrackerd ltserver_gencpp)
