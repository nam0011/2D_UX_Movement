cmake_minimum_required(VERSION 3.17)
project(DynMoveTake2)

set(CMAKE_CXX_STANDARD 14)

add_executable(
                DynMoveTake2
                            main.cpp
                            Agent.cpp Agent.h
                            path.cpp path.h
                            )