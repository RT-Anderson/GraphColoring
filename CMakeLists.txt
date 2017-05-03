cmake_minimum_required(VERSION 3.7)
project(GraphColoring)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp GraphColoringModule.cpp GraphColoringModule.h)
add_executable(GraphColoring ${SOURCE_FILES})