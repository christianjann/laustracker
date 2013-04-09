FILE (GLOB_RECURSE ueye_get_image_SRCS *.cpp *.cxx *.cc *.C *.c *.h *.hpp)

#MESSAGE("Found LAUSTRACKER_LIBRARY:")
#MESSAGE(${LAUSTRACKER_STATIC_LIBRARY})

set(ueye_get_image_LIBS ${Boost_LIBRARIES} 
                         ${LAUSTRACKER_SHARED_LIBRARY} 
                         ${UEyeCam_LIBRARIES})

set(ueye_get_image_BIN ${PROJECT_NAME}-ueye-get-image)

link_directories(${MAINFOLDER}/lib)

add_executable(${ueye_get_image_BIN} ${ueye_get_image_SRCS})
target_link_libraries(${ueye_get_image_BIN} ${ueye_get_image_LIBS})

install(TARGETS ${ueye_get_image_BIN} DESTINATION bin)

