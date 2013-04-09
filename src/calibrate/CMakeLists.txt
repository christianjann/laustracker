FILE (GLOB_RECURSE calibrate_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)


SET (calibrate_LIBS ${Boost_LIBRARIES} ${LAUSTRACKER_STATIC_LIBRARY})
SET (calibrate_BIN ${PROJECT_NAME}-calibrate)

LINK_DIRECTORIES(${MAINFOLDER}/lib)
ADD_EXECUTABLE(${calibrate_BIN} ${calibrate_SRCS})
TARGET_LINK_LIBRARIES(${calibrate_BIN} ${calibrate_LIBS})

INSTALL(TARGETS ${calibrate_BIN} DESTINATION bin)
