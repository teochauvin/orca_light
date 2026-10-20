cmake_minimum_required(VERSION 3.18)
project(rvo2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})

# find_package(CGAL REQUIRED COMPONENTS Core) 
find_package(pybind11 REQUIRED)

file(GLOB SOURCES "src/*.cpp" "bindings.cpp")

pybind11_add_module(rvo2 ${SOURCES})

target_include_directories(rvo2 PRIVATE "include" "src")
target_compile_options(rvo2 PRIVATE -O3 -frounding-math)



# performances monitoring 
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

target_compile_options(rvo2 PRIVATE 
    $<$<CONFIG:RelWithDebInfo>:-g -fno-omit-frame-pointer>
    $<$<CONFIG:Release>:-O3>
    -frounding-math
)



# Helper : to compile 

# mkdir build && cd build
# cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
# make

# valgrind --tool=callgrind --simulate-cache=yes python3 example3.py
