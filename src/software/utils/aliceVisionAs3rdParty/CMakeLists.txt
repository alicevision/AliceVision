cmake_minimum_required(VERSION 3.3)

project(AliceVisionAs3rdParty)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(AliceVision CONFIG REQUIRED)
message(STATUS "Found AliceVision : ${AliceVision_FOUND}")
message(STATUS "Found AliceVision version: ${AliceVision_VERSION}")

add_executable(testAV3rd main.cpp)
target_link_libraries(testAV3rd PUBLIC aliceVision_system)