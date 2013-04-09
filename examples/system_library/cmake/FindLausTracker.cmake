# Locate liblaustracker
#
# This module defines:
# LausTracker_FOUND, if false, do not try to link against liblaustracker
# LausTracker_LIBRARIES, path to the liblaustracker
# LausTracker_INCLUDE_DIR, where to find the laustracker header files
#
# Christian Jann <christian.jann@ymail.com>

IF(UNIX)

  IF(LausTracker_INCLUDE_DIR AND LausTracker_LIBRARIES)
    # in cache already
    SET(LausTracker_FIND_QUIETLY TRUE)
  ENDIF(LausTracker_INCLUDE_DIR AND LausTracker_LIBRARIES)

  FIND_PATH(LausTracker_INCLUDE_DIR
    NAMES laustracker/laustracker.h
    PATHS
    #/home/lasve/workspace/libueyecam/trunk
    /usr/local/include/laustracker
    /usr/include/laustracker
    ${LOCAL_LAUSTRACKER_PATH}/include
  )

  FIND_LIBRARY(LausTracker_LIBRARIES 
    NAMES laustracker
    PATHS
    #/home/lasve/workspace/libueyecam/trunk
    /usr/local/lib
    /usr/lib
    ${LOCAL_LAUSTRACKER_PATH}/lib
  )

  IF(LausTracker_LIBRARIES AND LausTracker_INCLUDE_DIR)
    SET(LausTracker_FOUND "YES")
    IF(NOT LausTracker_FIND_QUIETLY)
      MESSAGE(STATUS "Found liblaustracker: ${LausTracker_LIBRARIES}")
    ENDIF(NOT LausTracker_FIND_QUIETLY)
  ELSE(LausTracker_LIBRARIES AND LausTracker_INCLUDE_DIR)
    IF(NOT LausTracker_LIBRARIES)
      IF(LausTracker_FIND_REQUIRED)
        message(FATAL_ERROR "could not find liblaustracker.so")
      ENDIF(LausTracker_FIND_REQUIRED)
    ENDIF(NOT LausTracker_LIBRARIES)

    IF(NOT LausTracker_INCLUDE_DIR)
      IF(LausTracker_FIND_REQUIRED)
        message(FATAL_ERROR "could not find laustracker.h")
      ENDIF(LausTracker_FIND_REQUIRED)
    ENDIF(NOT LausTracker_INCLUDE_DIR)

  ENDIF(LausTracker_LIBRARIES AND LausTracker_INCLUDE_DIR)

ENDIF(UNIX)
 
