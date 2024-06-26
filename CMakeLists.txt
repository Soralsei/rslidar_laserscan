cmake_minimum_required (VERSION 2.8.3)
project (rslidar_laserscan)

# Set minimum C++ standard to C++11
if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message (STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message (STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set (CMAKE_CXX_STANDARD 11)
endif()

find_package (catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nodelet
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/RSLaserScan.cfg
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
)

include_directories (
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rslidar_laserscan.cpp
  src/nodelet.cpp
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)