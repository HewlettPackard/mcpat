add_library(cache
  datacache.h
  datacache.cc
  instcache.h
  instcache.cc
  sharedcache.h
  sharedcache.cc
)
target_include_directories(cache PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cache LINK_PUBLIC cacti top)
