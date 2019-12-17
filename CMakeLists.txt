cmake_minimum_required(VERSION 3.15)
project(Project2)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(Project2
        AStarPlanner.cpp
        AStarPlanner.h
        MAPFInstance.cpp
        MAPFInstance.h)
