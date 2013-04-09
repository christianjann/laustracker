# Copyright (c) 2013, Chemnitz University of Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the <organization> nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Locate libueye_api
# This module defines
# UEye_FOUND, if false, do not try to link to libeye_api
# UEye_LIBRARIES, path to the libeye_api
# UEye_INCLUDE_DIR, where to find the ueye.h header file

IF(UNIX)

  IF(UEye_INCLUDE_DIR AND UEye_LIBRARIES)
    # in cache already
    SET(UEye_FIND_QUIETLY TRUE)
  ENDIF(UEye_INCLUDE_DIR AND UEye_LIBRARIES)

  FIND_PATH(UEye_INCLUDE_DIR
    NAMES ueye.h
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES include
  )

  FIND_LIBRARY(UEye_LIBRARIES 
    NAMES ueye_api
    PATHS
    /usr/local
    /usr
    PATH_SUFFIXES lib
  )

  IF(UEye_LIBRARIES AND UEye_INCLUDE_DIR)
    SET(UEye_FOUND "YES")
    IF(NOT UEye_FIND_QUIETLY)
      MESSAGE(STATUS "Found libueye_api: ${UEye_LIBRARIES}")
    ENDIF(NOT UEye_FIND_QUIETLY)
  ELSE(UEye_LIBRARIES AND UEye_INCLUDE_DIR)
    IF(NOT UEye_LIBRARIES)
      IF(UEye_FIND_REQUIRED)
        message(FATAL_ERROR "could not find libueye_api.so")
      ENDIF(UEye_FIND_REQUIRED)
    ENDIF(NOT UEye_LIBRARIES)

    IF(NOT UEye_INCLUDE_DIR)
      IF(UEye_FIND_REQUIRED)
        message(FATAL_ERROR "could not find ueye.h")
      ENDIF(UEye_FIND_REQUIRED)
    ENDIF(NOT UEye_INCLUDE_DIR)

  ENDIF(UEye_LIBRARIES AND UEye_INCLUDE_DIR)

ENDIF(UNIX)
