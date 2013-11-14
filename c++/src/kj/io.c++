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

#include "io.h"
#include "debug.h"
#include <unistd.h>
#include <algorithm>
#include <errno.h>

#ifdef _WIN32
#include <windows.h>
#include <fcntl.h>
#include <io.h>
#else
#include <sys/uio.h>
#include <sys/socket.h>
#endif

namespace kj {

InputStream::~InputStream() noexcept(false) {}
OutputStream::~OutputStream() noexcept(false) {}
BufferedInputStream::~BufferedInputStream() noexcept(false) {}
BufferedOutputStream::~BufferedOutputStream() noexcept(false) {}

size_t InputStream::read(void* buffer, size_t minBytes, size_t maxBytes) {
  size_t n = tryRead(buffer, minBytes, maxBytes);
  KJ_REQUIRE(n >= minBytes, "Premature EOF") {
    // Pretend we read zeros from the input.
    memset(reinterpret_cast<byte*>(buffer) + n, 0, minBytes - n);
    return minBytes;
  }
  return n;
}

void InputStream::skip(size_t bytes) {
  char scratch[8192];
  while (bytes > 0) {
    size_t amount = std::min(bytes, sizeof(scratch));
    read(scratch, amount);
    bytes -= amount;
  }
}

void OutputStream::write(ArrayPtr<const ArrayPtr<const byte>> pieces) {
  for (auto piece: pieces) {
    write(piece.begin(), piece.size());
  }
}

ArrayPtr<const byte> BufferedInputStream::getReadBuffer() {
  auto result = tryGetReadBuffer();
  KJ_REQUIRE(result.size() > 0, "Premature EOF");
  return result;
}

// =======================================================================================

BufferedInputStreamWrapper::BufferedInputStreamWrapper(InputStream& inner, ArrayPtr<byte> buffer)
    : inner(inner), ownedBuffer(buffer == nullptr ? heapArray<byte>(8192) : nullptr),
      buffer(buffer == nullptr ? ownedBuffer : buffer) {}

BufferedInputStreamWrapper::~BufferedInputStreamWrapper() noexcept(false) {}

ArrayPtr<const byte> BufferedInputStreamWrapper::tryGetReadBuffer() {
  if (bufferAvailable.size() == 0) {
    size_t n = inner.tryRead(buffer.begin(), 1, buffer.size());
    bufferAvailable = buffer.slice(0, n);
  }

  return bufferAvailable;
}

size_t BufferedInputStreamWrapper::tryRead(void* dst, size_t minBytes, size_t maxBytes) {
  if (minBytes <= bufferAvailable.size()) {
    // Serve from current buffer.
    size_t n = std::min(bufferAvailable.size(), maxBytes);
    memcpy(dst, bufferAvailable.begin(), n);
    bufferAvailable = bufferAvailable.slice(n, bufferAvailable.size());
    return n;
  } else {
    // Copy current available into destination.
    memcpy(dst, bufferAvailable.begin(), bufferAvailable.size());
    size_t fromFirstBuffer = bufferAvailable.size();

    dst = reinterpret_cast<byte*>(dst) + fromFirstBuffer;
    minBytes -= fromFirstBuffer;
    maxBytes -= fromFirstBuffer;

    if (maxBytes <= buffer.size()) {
      // Read the next buffer-full.
      size_t n = inner.read(buffer.begin(), minBytes, buffer.size());
      size_t fromSecondBuffer = std::min(n, maxBytes);
      memcpy(dst, buffer.begin(), fromSecondBuffer);
      bufferAvailable = buffer.slice(fromSecondBuffer, n);
      return fromFirstBuffer + fromSecondBuffer;
    } else {
      // Forward large read to the underlying stream.
      bufferAvailable = nullptr;
      return fromFirstBuffer + inner.read(dst, minBytes, maxBytes);
    }
  }
}

void BufferedInputStreamWrapper::skip(size_t bytes) {
  if (bytes <= bufferAvailable.size()) {
    bufferAvailable = bufferAvailable.slice(bytes, bufferAvailable.size());
  } else {
    bytes -= bufferAvailable.size();
    if (bytes <= buffer.size()) {
      // Read the next buffer-full.
      size_t n = inner.read(buffer.begin(), bytes, buffer.size());
      bufferAvailable = buffer.slice(bytes, n);
    } else {
      // Forward large skip to the underlying stream.
      bufferAvailable = nullptr;
      inner.skip(bytes - bufferAvailable.size());
    }
  }
}

// -------------------------------------------------------------------

BufferedOutputStreamWrapper::BufferedOutputStreamWrapper(OutputStream& inner, ArrayPtr<byte> buffer)
    : inner(inner),
      ownedBuffer(buffer == nullptr ? heapArray<byte>(8192) : nullptr),
      buffer(buffer == nullptr ? ownedBuffer : buffer),
      bufferPos(this->buffer.begin()) {}

BufferedOutputStreamWrapper::~BufferedOutputStreamWrapper() noexcept(false) {
  unwindDetector.catchExceptionsIfUnwinding([&]() {
    flush();
  });
}

void BufferedOutputStreamWrapper::flush() {
  if (bufferPos > buffer.begin()) {
    inner.write(buffer.begin(), bufferPos - buffer.begin());
    bufferPos = buffer.begin();
  }
}

ArrayPtr<byte> BufferedOutputStreamWrapper::getWriteBuffer() {
  return arrayPtr(bufferPos, buffer.end());
}

void BufferedOutputStreamWrapper::write(const void* src, size_t size) {
  if (src == bufferPos) {
    // Oh goody, the caller wrote directly into our buffer.
    bufferPos += size;
  } else {
    size_t available = buffer.end() - bufferPos;

    if (size <= available) {
      memcpy(bufferPos, src, size);
      bufferPos += size;
    } else if (size <= buffer.size()) {
      // Too much for this buffer, but not a full buffer's worth, so we'll go ahead and copy.
      memcpy(bufferPos, src, available);
      inner.write(buffer.begin(), buffer.size());

      size -= available;
      src = reinterpret_cast<const byte*>(src) + available;

      memcpy(buffer.begin(), src, size);
      bufferPos = buffer.begin() + size;
    } else {
      // Writing so much data that we might as well write directly to avoid a copy.
      inner.write(buffer.begin(), bufferPos - buffer.begin());
      bufferPos = buffer.begin();
      inner.write(src, size);
    }
  }
}

// =======================================================================================

ArrayInputStream::ArrayInputStream(ArrayPtr<const byte> array): array(array) {}
ArrayInputStream::~ArrayInputStream() noexcept(false) {}

ArrayPtr<const byte> ArrayInputStream::tryGetReadBuffer() {
  return array;
}

size_t ArrayInputStream::tryRead(void* dst, size_t minBytes, size_t maxBytes) {
  size_t n = std::min(maxBytes, array.size());
  memcpy(dst, array.begin(), n);
  array = array.slice(n, array.size());
  return n;
}

void ArrayInputStream::skip(size_t bytes) {
  KJ_REQUIRE(array.size() >= bytes, "ArrayInputStream ended prematurely.") {
    bytes = array.size();
    break;
  }
  array = array.slice(bytes, array.size());
}

// -------------------------------------------------------------------

ArrayOutputStream::ArrayOutputStream(ArrayPtr<byte> array): array(array), fillPos(array.begin()) {}
ArrayOutputStream::~ArrayOutputStream() noexcept(false) {}

ArrayPtr<byte> ArrayOutputStream::getWriteBuffer() {
  return arrayPtr(fillPos, array.end());
}

void ArrayOutputStream::write(const void* src, size_t size) {
  if (src == fillPos) {
    // Oh goody, the caller wrote directly into our buffer.
    fillPos += size;
  } else {
    KJ_REQUIRE(size <= (size_t)(array.end() - fillPos),
            "ArrayOutputStream's backing array was not large enough for the data written.");
    memcpy(fillPos, src, size);
    fillPos += size;
  }
}

// =======================================================================================

AutoCloseFd::~AutoCloseFd() noexcept(false) {
  if (fd >= 0) {
    unwindDetector.catchExceptionsIfUnwinding([&]() {
      // Don't use SYSCALL() here because close() should not be repeated on EINTR.
      if (close(fd) < 0) {
        KJ_FAIL_SYSCALL("close", errno, fd) {
          break;
        }
      }
    });
  }
}

AutoCloseFile::~AutoCloseFile() noexcept(false) {
  if (file != NULL) {
    unwindDetector.catchExceptionsIfUnwinding([&]() {
      fclose(file);
    });
  }
}

FdInputStream::~FdInputStream() noexcept(false) {}

size_t FdInputStream::tryRead(void* buffer, size_t minBytes, size_t maxBytes) {
  char* pos = reinterpret_cast<char*>(buffer);
  char* min = pos + minBytes;
  char* max = pos + maxBytes;

  while (pos < min) {
    ssize_t n;
    KJ_SYSCALL(n = ::recv(fd, pos, max - pos, 0), fd);
    if (n == 0) {
      break;
    }
    pos += n;
  }

  return pos - reinterpret_cast<char*>(buffer);
}

FdOutputStream::~FdOutputStream() noexcept(false) {}

void FdOutputStream::write(const void* buffer, size_t size) {
  const char* pos = reinterpret_cast<const char*>(buffer);

  while (size > 0) {
    ssize_t n;
    KJ_SYSCALL(n = ::send(fd, pos, size, 0), fd);
    KJ_ASSERT(n > 0, "write() returned zero.");
    pos += n;
    size -= n;
  }
}

void FdOutputStream::write(ArrayPtr<const ArrayPtr<const byte>> pieces) {
#ifdef _WIN32
  OutputStream::write(pieces);
#else
  KJ_STACK_ARRAY(struct iovec, iov, pieces.size(), 16, 128);

  for (uint i = 0; i < pieces.size(); i++) {
    // writev() interface is not const-correct.  :(
    iov[i].iov_base = const_cast<byte*>(pieces[i].begin());
    iov[i].iov_len = pieces[i].size();
  }

  struct iovec* current = iov.begin();

  // Make sure we don't do anything on an empty write.
  while (current < iov.end() && current->iov_len == 0) {
    ++current;
  }

  while (current < iov.end()) {
    ssize_t n = 0;
    KJ_SYSCALL(n = ::writev(fd, current, iov.end() - current), fd);
    KJ_ASSERT(n > 0, "writev() returned zero.");

    while (n > 0 && static_cast<size_t>(n) >= current->iov_len) {
      n -= current->iov_len;
      ++current;
    }

    if (n > 0) {
      current->iov_base = reinterpret_cast<byte*>(current->iov_base) + n;
      current->iov_len -= n;
    }
  }
#endif
}

FileInputStream::~FileInputStream() noexcept(false) {}

void FileInputStream::setBinary() {
#ifdef _WIN32
  KJ_SYSCALL(_setmode(_fileno(file), _O_BINARY));
#else
  KJ_SYSCALL(freopen(NULL, "rb", file) != NULL);
#endif
}

size_t FileInputStream::tryRead(void* buffer, size_t minBytes, size_t maxBytes) {
  char* pos = reinterpret_cast<char*>(buffer);
  char* min = pos + minBytes;
  char* max = pos + maxBytes;

  while (pos < min) {
    size_t n = fread(pos, 1, max - pos, file);
    if (n == 0) {
      break;
    }
    pos += n;
  }

  return pos - reinterpret_cast<char*>(buffer);
}

FileOutputStream::~FileOutputStream() noexcept(false) {}

void FileOutputStream::setBinary() {
#ifdef _WIN32
  KJ_SYSCALL(_setmode(_fileno(file), _O_BINARY));
#else
  KJ_SYSCALL(freopen(NULL, "wb", file) != NULL);
#endif
}

void FileOutputStream::write(const void* buffer, size_t size) {
  const char* pos = reinterpret_cast<const char*>(buffer);

  while (size > 0) {
    size_t n = fwrite(pos, 1, size, file);
    KJ_ASSERT(n > 0, "write() returned zero.");
    pos += n;
    size -= n;
  }
}

#ifdef _WIN32
AutoCloseHandle::~AutoCloseHandle() noexcept(false) {
  if (handle != NULL && handle != INVALID_HANDLE_VALUE) {
    unwindDetector.catchExceptionsIfUnwinding([&]() {
      CloseHandle(handle);
    });
  }
}

HandleOutputStream::~HandleOutputStream() noexcept(false) {}

void HandleOutputStream::write(const void* buffer, size_t size) {
  const char* pos = reinterpret_cast<const char*>(buffer);

  while (size > 0) {
    DWORD n;
    KJ_WINCALL(WriteFile(h, pos, size, &n, NULL));
    pos += n;
    size -= n;
  }
}
#endif

}  // namespace kj
