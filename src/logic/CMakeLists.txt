add_library(logic
  dep_resource_conflict_check.h
  dep_resource_conflict_check.cc
  dff_cell.h
  dff_cell.cc
  functional_unit.h
  functional_unit.cc
  inst_decoder.h
  inst_decoder.cc
  pipeline.h
  pipeline.cc
  selection_logic.h
  selection_logic.cc
  undiff_core.h
  undiff_core.cc
)
target_include_directories(logic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(logic LINK_PUBLIC cacti top)
