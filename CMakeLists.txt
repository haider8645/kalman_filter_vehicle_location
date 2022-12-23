cmake_minimum_required(VERSION 3.9)
project(kalman_filter_robot_location
        VERSION 0.1
        DESCRIPTION "")

find_package(Eigen3 REQUIRED Dense)
find_package(Matplot++ REQUIRED)

set(HEADERS FilterPosition2D.hpp)
set(SOURCES FilterPosition2D.cpp)

add_library(kalman_filter_robot_location STATIC ${SOURCES})

add_subdirectory(matplotplusplus)
add_executable(kalman_filter_robot_location_bin Main.cpp)
target_include_directories(kalman_filter_robot_location_bin PUBLIC ${HEADERS})
target_link_libraries(kalman_filter_robot_location_bin PUBLIC 
                                      kalman_filter_robot_location 
                                      Matplot++::matplot
                                      Eigen3::Eigen)
