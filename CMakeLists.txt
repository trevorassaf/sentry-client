cmake_minimum_required(VERSION 3.1)

include_directories(
  src/
  ../../gpio14/repo/include/)

add_library(dynamixel STATIC
  src/dynamixel/AxA12.cpp
  src/dynamixel/AxA12BaudRate.cpp
  src/dynamixel/AxA12Factory.cpp
  src/dynamixel/AxA12InstructionPacket.cpp
  src/dynamixel/AxA12InstructionType.cpp
  src/dynamixel/AxA12ReaderWriter.cpp
  src/dynamixel/AxA12RegisterMap.cpp
  src/dynamixel/AxA12StatusPacket.cpp
  src/dynamixel/AxA12Utils.cpp)

target_link_libraries(dynamixel gflags::gflags)
target_link_libraries(dynamixel glog::glog)
target_link_libraries(dynamixel gpio14)

add_executable(test_dynamixel src/tests/dynamixel.cpp)

target_link_libraries(test_dynamixel gflags::gflags)
target_link_libraries(test_dynamixel glog::glog)
target_link_libraries(test_dynamixel gpio14)
target_link_libraries(test_dynamixel dynamixel)
