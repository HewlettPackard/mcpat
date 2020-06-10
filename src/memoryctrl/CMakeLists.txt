add_library(memoryctrl
  mc_backend.h
  mc_backend.cc
  mc_phy.h
  mc_phy.cc
)
target_include_directories(memoryctrl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(memoryctrl LINK_PUBLIC cacti top)
