FILE (GLOB_RECURSE undistort_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)


SET (undistort_LIBS ${Boost_LIBRARIES} ${LAUSTRACKER_STATIC_LIBRARY})
SET (undistort_BIN ${PROJECT_NAME}-undistort)

LINK_DIRECTORIES(${MAINFOLDER}/lib)
ADD_EXECUTABLE(${undistort_BIN} ${undistort_SRCS})
TARGET_LINK_LIBRARIES(${undistort_BIN} ${undistort_LIBS})

INSTALL(TARGETS ${undistort_BIN} DESTINATION bin)

