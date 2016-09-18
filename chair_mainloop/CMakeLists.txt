cmake_minimum_required (VERSION 2.6)
<<<<<<< HEAD
project (fsm)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PCL 1.6 REQUIRED)
=======
project (chair)

find_package(PCL 1.7 REQUIRED)
>>>>>>> b5a651a95cb2f1b0e93079e9f02ede92867e4504
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

<<<<<<< HEAD
add_executable(fsm planar_segmentation.cpp)
target_link_libraries(fsm ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
=======

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
add_executable(chair main.cpp machine.cpp)
target_link_libraries (chair ${PCL_LIBRARIES})
>>>>>>> b5a651a95cb2f1b0e93079e9f02ede92867e4504
