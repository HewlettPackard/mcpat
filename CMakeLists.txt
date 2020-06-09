if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "Create a separate build directory")
endif()

cmake_minimum_required (VERSION 3.1)

project(mcpat DESCRIPTION "Power Timing Area Calculator"
              LANGUAGES CXX)

set (MCPAT_VERSION_MAJOR 1)
set (MCPAT_VERSION_MINOR 3)
set (MCPAT_VERSION_PATCH 0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_PROFILE "-O3 -pg -g")
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

add_definitions(-DNTHREADS=1)

add_subdirectory(src)

