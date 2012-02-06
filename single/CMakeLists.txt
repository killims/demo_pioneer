cmake_minimum_required(VERSION 2.6)

project(single)

include(FindPkgConfig)

pkg_check_modules(player REQUIRED playerc++)
#find_package(OpenCV REQUIRED)
find_package( Boost 1.34.0 COMPONENTS program_options )



set (sources src/cell.cpp 
             src/config.cpp
             src/dstar.cpp
             src/dynamicmodel.cpp
             src/environment.cpp
             src/laser.cpp
             src/leader.cpp
             src/main.cpp
             src/map.cpp
             src/ndproxy.cpp
             src/robot.cpp
             src/utils.cpp)
             
set (ndsources  src/nd/calcul.cpp
                src/nd/geometria.cpp
                src/nd/nd.cpp
                src/nd/Scan.cpp)
                
add_library(nd STATIC ${ndsources})
add_executable(single ${sources})

find_package(PkConfig)

pkg_check_modules(OPENCV REQUIRED opencv)
if(OPENCV_FOUND)
	include_directories(${OPENCV_INCLUDE_DIRS})
	target_link_libraries(single ${OPENCV_LIBRARIES})
endif(OPENCV_FOUND)

add_definitions(-std=c++0x)
include_directories(src/nd ${player_INCLUDE_DIRS})
target_link_libraries(single nd ${player_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})




