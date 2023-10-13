include_directories(. /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)
add_executable(calibratemul calibratemul_test.cpp calibratemul.cpp ../simulator.cpp )

add_executable(decoupled_calib decoupled_calibration_micp.cpp calibratemul.cpp ../simulator.cpp ) #takes velocities from the file micp wtote to

add_executable(mul_traj_gen Multiple_Traj_generate.cpp ../simulator.cpp) 


