add_subdirectory(cacti)

add_executable(mcpat 
  arch_const.h
  array.h
  array.cc
  basic_components.h
  basic_components.cc
  core.h
  core.cc
  globalvar.h
  interconnect.h
  interconnect.cc
  iocontrollers.h
  iocontrollers.cc
  logic.h
  logic.cc
  noc.h
  noc.cc
  main.cc
  memoryctrl.h
  memoryctrl.cc
  options.h
  options.cc
  processor.h
  processor.cc
  sharedcache.h
  sharedcache.cc
  version.h
  xmlParser.h
  xmlParser.cc
  XML_Parse.h
  XML_Parse.cc
)

target_link_libraries(mcpat LINK_PUBLIC cacti Threads::Threads Boost::program_options)

add_custom_command(TARGET mcpat POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mcpat> ${PROJECT_BINARY_DIR}/mcpat
  COMMENT "Copying executable ${PROJECT_BINARY_DIR}/mcpat"
  )
