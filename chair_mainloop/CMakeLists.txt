cmake_minimum_required (VERSION 2.6)

project (chair)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(FHG_PCL_LIBS fhg_kdtree fhg_utils fhg_io fhg_common fhg_histogram fhg_types fhg_models fhg_segmentation fhg_opencv fhg_registration fhg_surface )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)
add_definitions(${PCL_DEFINITIONS})


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") // Has running problem with YHY's program

add_executable(chair main.cpp machine.cpp)
target_link_libraries (chair ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp ${FHG_PCL_LIBS})

message("${PCL_INCLUDE_DIRS}")
message("${PCL_LIBRARIES}")
