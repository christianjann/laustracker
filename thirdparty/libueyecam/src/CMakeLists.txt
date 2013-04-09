cmake_minimum_required(VERSION 2.6)

#--> Dependencies, OpenCV
FIND_PACKAGE(OpenCV REQUIRED)                                                     # For OpenCV only
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR} ${HIGHGUI_INCLUDE_DIR})    # For OpenCV only
IF(DEFINED OpenCV_LIBRARIES)
  SET(LINK_LIBRARIES ${LINK_LIBRARIES} ${OpenCV_LIBRARIES}) # for compatibility with cmake-data-proaut
ELSE(DEFINED OpenCV_LIBRARIES)
  SET(LINK_LIBRARIES ${LINK_LIBRARIES} ${OpenCV_LIBS}) # this should be the default upstream name
ENDIF(DEFINED OpenCV_LIBRARIES)
#<-- Dependencies, OpenCV

#--> Dependencies, uEye_api
FIND_PACKAGE(UEye REQUIRED)                                                       # For using uEye Camera only
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${UEye_INCLUDE_DIR})                             # For using uEye Camera only
SET(LINK_LIBRARIES ${LINK_LIBRARIES} ${UEye_LIBRARIES})
#<-- Dependencies, uEye_api

SET(INCLUDE_DIRS ${INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include")
MESSAGE("${INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Create a library
ADD_LIBRARY(ueyecam SHARED ueyecam.cpp)
set(UEYECAM_SHARED_LIBRARY ueyecam CACHE INTERNAL "ueyecam shared library")

# Link the library against its dependencies
TARGET_LINK_LIBRARIES(ueyecam ${LINK_LIBRARIES})

# version
SET_TARGET_PROPERTIES(ueyecam PROPERTIES VERSION 0.0.1 SOVERSION 1)

INSTALL(TARGETS ueyecam LIBRARY DESTINATION lib)
