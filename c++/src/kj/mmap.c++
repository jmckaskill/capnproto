// Copyright (c) 2013, Kenton Varda <temporal@gmail.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "mmap.h"
#include "io.h"
#include "debug.h"
#include "vector.h"
#include <stdint.h>
#include <sys/mman.h>

namespace kj {

class MmapDisposer: public kj::ArrayDisposer {
protected:
  void disposeImpl(void* firstElement, size_t elementSize, size_t elementCount,
                   size_t capacity, void (*destroyElement)(void*)) const {
    munmap(firstElement, elementSize * elementCount);
  }
};

constexpr MmapDisposer mmapDisposer = MmapDisposer();

Array<const char> mmapForRead(StringPtr filename) {
  int fd;
  // We already established that the file exists, so this should not fail.
  KJ_SYSCALL(fd = open(filename.cStr(), O_RDONLY), filename);
  kj::AutoCloseFd closer(fd);

  struct stat stats;
  KJ_SYSCALL(fstat(fd, &stats));

  if (S_ISREG(stats.st_mode)) {
    if (stats.st_size == 0) {
      // mmap()ing zero bytes will fail.
      return nullptr;
    }

    // Regular file.  Just mmap() it.
    const void* mapping = mmap(NULL, stats.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (mapping == MAP_FAILED) {
      KJ_FAIL_SYSCALL("mmap", errno, filename);
    }

    return kj::Array<const char>(
        reinterpret_cast<const char*>(mapping), stats.st_size, mmapDisposer);
  } else {
    // This could be a stream of some sort, like a pipe.  Fall back to read().
    // TODO(cleanup):  This does a lot of copies.  Not sure I care.
    kj::Vector<char> data(8192);

    char buffer[4096];
    for (;;) {
      ssize_t n;
      KJ_SYSCALL(n = read(fd, buffer, sizeof(buffer)));
      if (n == 0) break;
      data.addAll(buffer, buffer + n);
    }

    return data.releaseAsArray();
  }
}

} // namespace kj
