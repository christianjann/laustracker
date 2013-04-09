cmake_minimum_required(VERSION 2.6)
project (UEyeCam)

# run eclipse make once with this option on for autodetecting the include directories
SET(CMAKE_VERBOSE_MAKEFILE ON)
# for debugging
SET(CMAKE_BUILD_TYPE debug)
#SET(CMAKE_BUILD_TYPE release)

# Create the lib/ subdirectory, needed for build output
FILE(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(UEYECAM_LIBRARY_DIRS ${LIBRARY_OUTPUT_PATH} CACHE INTERNAL "ueyecam library output path")
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

INSTALL(FILES include/ueyecam.h DESTINATION include)

# Set the Root directory, which is searched preferentially
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

# Installation rule for the find scripts
SET(ADDITIONAL_INSTALL_FILES "${CMAKE_SOURCE_DIR}/cmake/Modules/FindUEyeCam.cmake")
# INSTALL(FILES ${ADDITIONAL_INSTALL_FILES} DESTINATION "/usr/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules")
INSTALL(FILES ${ADDITIONAL_INSTALL_FILES} DESTINATION "share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(examples)
