cmake_minimum_required(VERSION 2.8.3)
project(lesson_6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#tie to cmake in the subdirecticories
add_subdirectory("multiple v calibrator") 
add_subdirectory("single v calibrator")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)


#path to Eigen for ubuntu
#include_directories(. /usr/include/eigen3)

include_directories(. /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)
#path to Eigen for macOS

add_executable(sim mysimtest.cpp simulator.cpp) #1 pair(v,w) velocity simulator
add_executable(sim_mul mysimtest_multiple_velocities.cpp simulator.cpp) #3 pairs of (v,w)
add_executable(gen_sim_mul mysimtest_generic_multiple_velocities.cpp simulator.cpp) #generic number of pairs even though I use 5
add_executable(validate eigen_micp_2d_utils.cpp eigen_micp_2d.cpp myvalidate.cpp ) #estimate the velocities between two scans taken in each pose
add_executable(noisy_validate eigen_micp_2d_utils.cpp eigen_micp_2d.cpp noisy_micp.cpp )
add_executable(conn_validate eigen_micp_2d_utils.cpp eigen_micp_2d.cpp myvalidate_connected_scans.cpp ) # estimates the velocities between each scan and the next one indepedently of pose 
add_executable(coupled_calib eigen_micp_2d_utils.cpp coupled_calibrator.cpp coupled_calibrate_test.cpp ) # estimates the the kinematic paramters for each velocity between each scan and the next one continuosly. 

add_executable(noisy_coupled_calib eigen_micp_2d_utils.cpp coupled_calibrator.cpp Noise_coupled_calibrate_test.cpp )

add_executable(Traj_verify Trajec_verify.cpp simulator.cpp)
add_executable(optimal_traj_verify optimal_trajec_verify.cpp simulator.cpp)


