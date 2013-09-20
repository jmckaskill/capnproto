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

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#endif

namespace kj {

#ifdef _WIN32

class WinUnmap: public kj::ArrayDisposer {
public:
  WinUnmap(AutoCloseHandle&& hmap, AutoCloseHandle&& hfile): hmap(mv(hmap)), hfile(mv(hfile)) {}
  KJ_DISALLOW_COPY(WinUnmap);
  ~WinUnmap() noexcept(false) {}

protected:
  void disposeImpl(void* firstElement, size_t elementSize, size_t elementCount,
                   size_t capacity, void (*destroyElement)(void*)) const override {
    KJ_WINCALL(UnmapViewOfFile(firstElement));
    delete const_cast<WinUnmap*>(this);
  }

private:
  AutoCloseHandle hmap;
  AutoCloseHandle hfile;
};

Array<const char> mmapForRead(StringPtr filename) {
  AutoCloseHandle hfile(CreateFileA(filename.cStr(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL));
  KJ_WINCALL(hfile.get() != INVALID_HANDLE_VALUE);

  DWORD highsz;
  DWORD lowsz = GetFileSize(hfile, &highsz);
  if (lowsz != INVALID_FILE_SIZE) {
    if (lowsz == 0 && highsz == 0) {
      return nullptr;
    }

    AutoCloseHandle hmap(CreateFileMapping(hfile, NULL, PAGE_READONLY, highsz, lowsz, NULL));
    KJ_WINCALL(hmap.get() != INVALID_HANDLE_VALUE);

    size_t sz = (size_t) ((uint64_t(highsz) << 32) | uint64_t(lowsz));
    void* data = MapViewOfFile(hmap, FILE_MAP_READ, 0, 0, sz);
    KJ_WINCALL(data != NULL);

    return kj::Array<const char>(reinterpret_cast<const char*>(data), sz, *(new WinUnmap(mv(hmap), mv(hfile))));
  } else {
    // This could be a stream of some sort, like a pipe.  Fall back to read().
    // TODO(cleanup):  This does a lot of copies.  Not sure I care.
    kj::Vector<char> data(8192);

    char buffer[4096];
    for (;;) {
      DWORD n;
      KJ_WINCALL(ReadFile(hfile, buffer, sizeof(buffer), &n, NULL));
      if (n == 0) break;
      data.addAll(buffer, buffer + n);
    }

    return data.releaseAsArray();
  }
}

#else // _WIN32

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

#endif // _WIN32

} // namespace kj
