cmake_minimum_required(VERSION 3.21)
project(Eigen)

set(CMAKE_CXX_STANDARD 17)

add_executable(main matrix_le.cpp plot_trajecotires.cpp quaternion_R_v.cpp 2robots_coords.cpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

target_link_libraries(main Eigen3::Eigen)
target_link_libraries(main  ${Pangolin_LIBRARIES})