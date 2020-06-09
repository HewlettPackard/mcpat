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

add_custom_command(TARGET mcpat POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mcpat> ${PROJECT_BINARY_DIR}/mcpat
  COMMENT "Copying executable ${PROJECT_BINARY_DIR}/mcpat"
  )
