FILE (GLOB_RECURSE video-record_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)


SET (video-record_LIBS ${Boost_LIBRARIES} ${LAUSTRACKER_SHARED_LIBRARY} ${UEyeCam_LIBRARIES})
SET (video-record_BIN ${PROJECT_NAME}-video-record)

LINK_DIRECTORIES(${MAINFOLDER}/lib)
ADD_EXECUTABLE(${video-record_BIN} ${video-record_SRCS})
TARGET_LINK_LIBRARIES(${video-record_BIN} ${video-record_LIBS} ${RT_LIBRARY})

INSTALL(TARGETS ${video-record_BIN} DESTINATION bin)

