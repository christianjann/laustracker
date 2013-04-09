cmake_minimum_required(VERSION 2.8)

project( system_library )

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_PREFIX}")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package( OpenCV REQUIRED )
find_package( UEyeCam REQUIRED )
find_package( LausTracker REQUIRED )
include_directories(${LausTracker_INCLUDE_DIR})

add_executable( system_library main.cpp )
target_link_libraries( system_library ${LausTracker_LIBRARIES} 
                                      ${OpenCV_LIBS} 
                                      ${UEyeCam_LIBRARIES})
