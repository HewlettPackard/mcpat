add_library(iocontrollers
  flashcontroller.h
  flashcontroller.cc
)
target_include_directories(iocontrollers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(iocontrollers LINK_PUBLIC cacti top)
