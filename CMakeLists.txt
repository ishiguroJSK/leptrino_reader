cmake_minimum_required(VERSION 2.8.3)
project(leptrino_reader)
find_package(catkin REQUIRED COMPONENTS geometry_msgs
					#orocos_kdl
					roscpp
					roslib
					tf)

catkin_package()

include_directories(${catkin_INCLUDEDIR})
add_executable(leptrino_reader
    src/leptrino_reader.cpp
    src/rs_comm.cpp
)
target_link_libraries(leptrino_reader ${catkin_LIBRARIES})

#install(TARGETS leptrino_reader RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(FILES leptrino_reader.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
