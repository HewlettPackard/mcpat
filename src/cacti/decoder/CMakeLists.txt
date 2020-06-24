add_library(decoder
  decoder.h
  decoder.cc
  driver.h
  driver.cc
  predec.h
  predec.cc
  predec_blk.h
  predec_blk.cc
  predec_blk_drv.h
  predec_blk_drv.cc
)
target_include_directories(decoder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(decoder
                      LINK_PUBLIC
                      cacti
                      top)
