add_library(core
  branch_predictor.h
  branch_predictor.cc
  core.h
  core.cc
  exec_unit.h
  exec_unit.cc
  instfetch.h
  instfetch.cc
  loadstore.h
  loadstore.cc
  mmu.h
  mmu.cc
  regfile.h
  regfile.cc
  renaming_unit.h
  renaming_unit.cc
  scheduler.h
  scheduler.cc
)
target_include_directories(core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(core LINK_PUBLIC cacti cache top)
