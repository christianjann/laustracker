# - Try to find the liblog4cxx libraries
# Once done this will define
#
# LOG4CXX_FOUND - system has liblog4cxx
# Log4Cxx_INCLUDE_DIR - the liblog4cxx include directory
# Log4Cxx_LIBRARY - liblog4cxx library

FIND_PATH(LOG4CXX_INCLUDE_DIR logger.h PATHS /usr/include/log4cxx /usr/local/include/log4cxx )
FIND_LIBRARY(LOG4CXX_LIBRARIES NAMES log4cxx )

IF(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)
  SET(LOG4CXX_FOUND 1)
  #remove last /log4cxx string
  STRING(REGEX REPLACE "/log4cxx" "" LOG4CXX_INCLUDE_DIR_SUP_LEVEL ${LOG4CXX_INCLUDE_DIR})
  SET (Log4Cxx_INCLUDE_DIR ${LOG4CXX_INCLUDE_DIR_SUP_LEVEL} ${LOG4CXX_INCLUDE_DIR} )
  SET (Log4Cxx_LIBRARY ${LOG4CXX_LIBRARIES})
  if(NOT LOG4CXX_FIND_QUIETLY)
   message(STATUS "Found log4cxx: ${LOG4CXX_LIBRARIES}")
  endif(NOT LOG4CXX_FIND_QUIETLY)
ELSE(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)
  SET(LOG4CXX_FOUND 0 CACHE BOOL "Not found log4cxx library")
  message(STATUS "NOT Found log4cxx, disabling it")
ENDIF(LOG4CXX_INCLUDE_DIR AND LOG4CXX_LIBRARIES)
