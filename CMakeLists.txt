cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(project-name2)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-function")