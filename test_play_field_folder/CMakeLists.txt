cmake_minimum_required(VERSION 2.8.3)
project(lesson_6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)


include_directories(. /usr/include/eigen3)
add_executable(sim mysimtest.cpp simulator.cpp)
add_executable(sim1 mysimtest_c.cpp simulator.cpp)
add_executable(validate eigen_micp_2d_utils.cpp eigen_micp_2d.cpp myvalidate.cpp )



