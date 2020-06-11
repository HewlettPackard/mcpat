#include "instcache.h"

#include <iostream>
#include <string>

InstCache::InstCache() {
  caches = nullptr;
  missb = nullptr;
  ifb = nullptr;
  prefetchb = nullptr;
};

InstCache::~InstCache() {
  if (caches) { // caches->local_result.cleanup();
    delete caches;
    caches = 0;
  }
  if (missb) { // missb->local_result.cleanup();
    delete missb;
    missb = 0;
  }
  if (ifb) { // ifb->local_result.cleanup();
    delete ifb;
    ifb = 0;
  }
  if (prefetchb) { // prefetchb->local_result.cleanup();
    delete prefetchb;
    prefetchb = 0;
  }
};
