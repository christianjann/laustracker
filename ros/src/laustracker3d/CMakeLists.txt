cmake_minimum_required(VERSION 2.8)
project(laustracker3d)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/share/cmake)

set(CMAKE_CXX_FLAGS "-std=c++0x")

include(ecl_platform_detection)
ecl_detect_distro()
if(${DISTRO_NAME} MATCHES "Ubuntu")
message("-- Building Laustracker 3D on Ubuntu ${DISTRO_VERSION}
        ${DISTRO_VERSION_STRING}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUBUNTU")
endif(${DISTRO_NAME} MATCHES "Ubuntu")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg
                                        cv_bridge image_transport)
find_package(OpenCV REQUIRED)

## Declare a catkin package
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(Boost_ADDITIONAL_VERSIONS "1.39" "1.39.0" "1.40" "1.40.0" "1.41" "1.41.0"
"1.42" "1.42.0" "1.43" "1.43.0" "1.44" "1.44.0" "1.45" "1.45.0" "1.46" "1.46.0"
"1.47" "1.47.0" "1.48" "1.48.0" "1.49" "1.49.0" "1.50" "1.50.0")
find_package(Boost 1.37 COMPONENTS "date_time" "filesystem" "graph" "iostreams"
              "program_options" "regex" "serialization" "signals" "system"
              "thread" "wserialization" REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(OpenSceneGraph REQUIRED OpenThreads osgDB osgGA osgUtil osgSim
                                     osgViewer osgText)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

set(LOCAL_LAUSTRACKER_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
find_package( LausTracker REQUIRED )
include_directories(${LausTracker_INCLUDE_DIR})


add_executable(laustracker3d laustracker3D.cpp)
target_link_libraries(laustracker3d ${Boost_LIBRARIES}
                                    ${LausTracker_LIBRARIES}
                                    ${OPENSCENEGRAPH_LIBRARIES}
                                    ${OPENGL_LIBRARIES}
                                    ${catkin_LIBRARIES})
