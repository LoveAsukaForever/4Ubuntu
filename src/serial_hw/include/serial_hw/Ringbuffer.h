/**
 * @file Ringbuffer.h
 * @author Keten (2863861004@qq.com)
 * @brief 用于串口接收的环形缓冲区
 * @version 0.1
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstring>

namespace serial_hw {

class RingBuffer {

private:
  uint8_t *buffer;
  size_t capacity;
  size_t mask;
  std::atomic<size_t> head{0};
  std::atomic<size_t> tail{0};

public:
  explicit RingBuffer(size_t size = 8192);
  ~RingBuffer();

  size_t write(const uint8_t *data, size_t len);
  size_t read(uint8_t *dest, size_t len);
  size_t peek(uint8_t *dest, size_t len) const;
  size_t available() const;
  size_t free_space() const;
  void clear();
  bool empty() const;
  bool full() const;
};
} // namespace serial_hw
