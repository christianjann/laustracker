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

# Locate libueyecam
#
# This module defines:
# UEyeCam_FOUND, if false, do not try to link against libeyecam
# UEyeCam_LIBRARIES, path to the libeyecam
# UEyeCam_INCLUDE_DIR, where to find the ueyecam.h header file

IF(UNIX)

  # Set the Root directory, which is searched preferentially
  SET(UEyeCam_ROOT "${UEyeCam_ROOT}" CACHE PATH "Root directory to search for libueyecam")

  # If the root directory changed, we need to search again for the libraries!
  # In order to do so, we need to unset the path variables for the library.
  IF(NOT(DEFINED UEyeCam_ROOT_LAST))
    SET(UEyeCam_ROOT_LAST "NotAnOption" CACHE STRING "last library loaded")
    MARK_AS_ADVANCED (FORCE UEyeCam_ROOT_LAST)
  ENDIF()
  IF(NOT (${UEyeCam_ROOT} MATCHES ${UEyeCam_ROOT_LAST}))
    UNSET(UEyeCam_INCLUDE_DIR CACHE)
    UNSET(UEyeCam_LIBRARIES CACHE)
    UNSET(UEyeCam_FIND_QUIETLY CACHE)
    SET(UEyeCam_ROOT_LAST ${UEyeCam_ROOT} CACHE STRING "Updating Library Project Configuration Option" FORCE)
  ENDIF()

  IF(UEyeCam_INCLUDE_DIR AND UEyeCam_LIBRARIES)
    # in cache already
    SET(UEyeCam_FIND_QUIETLY TRUE)
  ENDIF(UEyeCam_INCLUDE_DIR AND UEyeCam_LIBRARIES)

  FIND_PATH(UEyeCam_INCLUDE_DIR
    NAMES 
    ueyecam.h
    PATH_SUFFIXES
    include
    HINTS
    "${UEyeCam_ROOT}"
    )

  FIND_LIBRARY(UEyeCam_LIBRARIES
    NAMES ueyecam
    PATH_SUFFIXES
    lib
    HINTS
    "${UEyeCam_ROOT}")

  IF(UEyeCam_LIBRARIES AND UEyeCam_INCLUDE_DIR)
    SET(UEyeCam_FOUND "YES")
    IF(NOT UEyeCam_FIND_QUIETLY)
      MESSAGE(STATUS "Found libueyecam: ${UEyeCam_LIBRARIES}")
    ENDIF(NOT UEyeCam_FIND_QUIETLY)
  ELSE(UEyeCam_LIBRARIES AND UEyeCam_INCLUDE_DIR)
    IF(NOT UEyeCam_LIBRARIES)
      IF(UEyeCam_FIND_REQUIRED)
        message(FATAL_ERROR "could not find libueyecam.so")
      ENDIF(UEyeCam_FIND_REQUIRED)
    ENDIF(NOT UEyeCam_LIBRARIES)

    IF(NOT UEyeCam_INCLUDE_DIR)
      IF(UEyeCam_FIND_REQUIRED)
        message(FATAL_ERROR "could not find ueyecam.h")
      ENDIF(UEyeCam_FIND_REQUIRED)
    ENDIF(NOT UEyeCam_INCLUDE_DIR)

  ENDIF(UEyeCam_LIBRARIES AND UEyeCam_INCLUDE_DIR)

ENDIF(UNIX)
