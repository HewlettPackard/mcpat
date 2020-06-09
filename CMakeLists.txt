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

#Including custom CMake rules
include(cmake/clang-cxx-dev-tools.cmake)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)

add_definitions(-DNTHREADS=1)

add_subdirectory(cacti)

add_executable(mcpat 
  XML_Parse.h
  arch_const.h
  array.h
  basic_components.h
  core.h
  globalvar.h
  interconnect.h
  iocontrollers.h
  logic.h
  memoryctrl.h
  noc.h
  processor.h
  sharedcache.h
  version.h
  xmlParser.h
  XML_Parse.cc
  array.cc
  basic_components.cc
  core.cc
  interconnect.cc
  iocontrollers.cc
  logic.cc
  main.cc
  memoryctrl.cc
  noc.cc
  processor.cc
  sharedcache.cc
  xmlParser.cc
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(mcpat LINK_PUBLIC cacti Threads::Threads)
