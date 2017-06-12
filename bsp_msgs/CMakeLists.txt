cmake_minimum_required(VERSION 2.8.3)
project(bsp_msgs)

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

find_package(catkin REQUIRED 
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  octomap_msgs
  )

add_message_files(
  DIRECTORY msg
  FILES	ROVIO_Point2fMsg.msg
	ROVIO_RobocentricFeatureElementMsg.msg
 	ROVIO_FeatureSetManagerMsg.msg
 	ROVIO_StateMsg.msg
 	ROVIO_FilterStateMsg.msg
	BSP_TrajectoryReferenceMsg.msg
)

add_service_files(
  FILES
  BSP_SrvSendFilterState.srv
  BSP_SrvPropagateFilterState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  octomap_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  octomap_msgs
  )
