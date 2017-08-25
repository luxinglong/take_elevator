cmake_minimum_required(VERSION 2.6)
project("AutoTakeElevator")

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
set (CMAKE_CXX_STANDARD 11)

find_package(Pangolin 0.4 REQUIRED)
find_package(OpenCV REQUIRED)

#set(LINK_DIR ${PROJECT_SOURCE_DIR}/lib)
#link_directories(${LINK_DIR})

include_directories(
	${Pangolin_INCLUDE_DIRS} 
	${PROJECT_SOURCE_DIR}/include)

add_executable(rplidar_viewer src/rplidar_viewer.cpp)
add_executable(testjson test/testJson.cpp)

target_link_libraries(rplidar_viewer
	${OpenCV_LIBS} 
	${Pangolin_LIBRARIES} 
	${PROJECT_SOURCE_DIR}/lib/librplidar_sdk.a)
