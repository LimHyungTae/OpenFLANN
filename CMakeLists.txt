cmake_minimum_required(VERSION 3.17)
project(OpenFLANN)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)

add_executable(kdtree_benchmark src/main.cpp)
target_link_libraries(kdtree_benchmark ${PCL_LIBRARIES})

add_executable(kdtree_nanoflann src/nanoflann_example.cpp)
target_link_libraries(kdtree_nanoflann ${PCL_LIBRARIES})

add_executable(kdtree_picoflann src/picoflann_example.cpp)
target_link_libraries(kdtree_picoflann ${PCL_LIBRARIES})
