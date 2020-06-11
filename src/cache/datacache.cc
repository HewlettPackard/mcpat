#include "datacache.h"

#include <iostream>
#include <string>

DataCache::DataCache() { 
  wbb = nullptr;
};

DataCache::~DataCache() {
  if (wbb) { 
    // wbb->local_result.cleanup();
    delete wbb;
    wbb = 0;
  }
};
