cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ymbc_tutorial)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")

# find pcl package
find_package(PCL 1.6 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include_directories(/usr/local/include/mylib1 /usr/local/include/mylib2)
#link_directories(/usr/local/lib/mylib1 /usr/local/lib/mylib2)
#add_definitions(-DMYDEFINITION1 -DMYDEFINITION2)

# set the executable name, sources, libraries
add_executable(spur_tutorial src/spur_tutorial.cpp)
target_link_libraries(spur_tutorial ypspur)

add_executable(spur_tutorial_ver2.0 src/spur_tutorial_ver2.0.cpp)
target_link_libraries(spur_tutorial_ver2.0 ypspur)

add_executable(laser_tutorial src/laser_tutorial.cpp)
target_link_libraries(laser_tutorial ypspur scip2awd pthread m)

add_executable(ex_final src/ex_final.cpp)
target_link_libraries(ex_final ypspur scip2awd pthread m ${PCL_LIBRARIES})
