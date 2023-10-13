include_directories(. /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3)
add_executable(calibrate calibrator_test.cpp calibrator.cpp ../simulator.cpp )
add_executable(calibratev calibrator_test.cpp calibrator_v.cpp ../simulator.cpp )
