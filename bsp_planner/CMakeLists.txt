cmake_minimum_required(VERSION 2.8.3)
project(bsp_planner)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  message_generation
  octomap_world
  tf
)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

add_service_files(
  FILES
  bsp_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
  LIBRARIES nbvplanner ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs visualization_msgs octomap_world tf kdtree
)

include_directories( include
		     thirdparty
  		     ${catkin_INCLUDE_DIRS}
  		     ${Eigen3_INCLUDE_DIRS}
  		     ${OCTOMAP_INCLUDE_DIRS} )

# Library
add_library(${PROJECT_NAME} src/bsp.cpp thirdparty/kdtree/kdtree.c)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
				      ${OCTOMAP_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg
				 ${${PROJECT_NAME}_EXPORTED_TARGETS})
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Node
add_executable(${PROJECT_NAME}_exe src/bsp_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})
SET_TARGET_PROPERTIES(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_exe DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
