cmake_minimum_required(VERSION 3.1)

include_directories(
  src/
  ../../gpio14/repo/include/)

add_executable(dynamixel
  src/tests/dynamixel.cpp
  src/dynamixel/AxA12RegisterMap.cpp)

target_link_libraries(dynamixel gflags::gflags)
target_link_libraries(dynamixel glog::glog)
target_link_libraries(dynamixel gpio14)
