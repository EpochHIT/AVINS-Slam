cmake_minimum_required(VERSION 2.8.3)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Force use of apt Ceres 1.14.0 and avoid picking /usr/local (Ceres 2.x)
set(CERES_HINT_DIRS "/usr/lib/cmake/Ceres" "/lib/cmake/Ceres")
find_package(Ceres 1.14.0 EXACT REQUIRED CONFIG PATHS ${CERES_HINT_DIRS} NO_DEFAULT_PATH)
message(STATUS "Ceres 1.14.0 from: ${Ceres_DIR}")

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(BEFORE ${CERES_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)


catkin_package()


add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} libGeographiccc) 