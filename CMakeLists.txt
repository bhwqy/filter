cmake_minimum_required(VERSION 3.0)
project(qyfilter)
include_directories(${qyfilter_SOURCE_DIR})

find_package(Eigen3 CONFIG REQUIRED)

add_library(ekf src/ekf.cpp)
target_link_libraries(ekf PRIVATE Eigen3::Eigen)
target_compile_features(ekf PUBLIC cxx_std_11)

add_library(particle src/particle.cpp)
target_link_libraries(particle PRIVATE Eigen3::Eigen)
target_compile_features(particle PUBLIC cxx_std_11)

add_executable(test_pf test/test_pf.cpp)
target_link_libraries(test_pf PRIVATE Eigen3::Eigen particle)
