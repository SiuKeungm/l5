cmake_minimum_required(VERSION 3.8)
project(l5)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o2 -pthread")
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS})

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES})

add_executable(GN GN-BA.cpp)
target_link_libraries(GN ${Sophus_LIBRARIES})

add_executable(draw draw.cpp)
target_link_libraries(draw ${Pangolin_LIBRARIES}
                           ${Sophus_LIBRARIES} ${OpenCV_LIBS})
