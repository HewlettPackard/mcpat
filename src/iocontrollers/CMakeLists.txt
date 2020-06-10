add_library(iocontrollers
  flash_controller.h
  flash_controller.cc
  pcie_controller.h
  pcie_controller.cc
  niu_controller.h
  niu_controller.cc
)
target_include_directories(iocontrollers PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(iocontrollers LINK_PUBLIC cacti top)
