cmake_minimum_required(VERSION 3.10.0)

project(LinearInterpolator)
find_package(RobWork REQUIRED)

add_executable(linear2 linear2.cpp)
target_link_libraries(linear2 ${ROBWORK_LIBRARIES})


