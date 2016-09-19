cmake_minimum_required (VERSION 2.6)

project (chair)

find_package(PCL 1.7 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") // Has running problem with YHY's program
add_executable(chair main.cpp machine.cpp)
target_link_libraries (chair ${PCL_LIBRARIES})

message("${PCL_INCLUDE_DIRS}")
message("${PCL_LIBRARIES}")