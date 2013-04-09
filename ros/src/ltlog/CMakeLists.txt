cmake_minimum_required(VERSION 2.8)
project(ltlog)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation
             ltserver)

catkin_python_setup()

generate_messages(DEPENDENCIES std_msgs ltserver)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs ltserver)

install(PROGRAMS ltlog.py plot-log.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
