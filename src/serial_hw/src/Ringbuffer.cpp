/**
 * @file Ringbuffer.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 用于串口接收的环形缓冲区
 * @version 0.1
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 环形缓冲区大小一般设计为2的幂，方便通过位运算快速计算索引，提高性能
 * @versioninfo :
 */
#include <serial_hw/Ringbuffer.h>

namespace serial_hw {

RingBuffer::RingBuffer(size_t size) {
  capacity = size;
  if ((capacity & (capacity - 1)) != 0) {
    capacity = 1 << (32 - __builtin_clz(capacity - 1)); // 向上取整到2的幂
  }
  mask = capacity - 1;
  buffer = new uint8_t[capacity]{0};
}

// 释放动态分配的缓冲区内存，防止内存泄漏
RingBuffer::~RingBuffer() { delete[] buffer; }

// 写入数据到缓冲区
size_t RingBuffer::write(const uint8_t *data, size_t len) {
  size_t free = free_space();
  len = std::min(len, free);

  if (len == 0)
    return 0;

  size_t h = head.load(std::memory_order_relaxed);
  size_t to_end = capacity - h;

  if (len <= to_end) {
    std::memcpy(buffer + h, data, len);
  } else {
    std::memcpy(buffer + h, data, to_end);
    std::memcpy(buffer, data + to_end, len - to_end);
  }

  head.store((h + len) & mask, std::memory_order_release);
  return len;
}

// 从缓冲区读走数据
size_t RingBuffer::read(uint8_t *dest, size_t len) {
  size_t avail = available();
  len = std::min(len, avail);
  if (len == 0)
    return 0;

  size_t t = tail.load(std::memory_order_relaxed);
  size_t to_end = capacity - t;

  if (len <= to_end) {
    std::memcpy(dest, buffer + t, len);
  } else {
    std::memcpy(dest, buffer + t, to_end);
    std::memcpy(dest + to_end, buffer, len - to_end);
  }

  tail.store((t + len) & mask, std::memory_order_release);
  return len;
}

// 查看数据，但不读出
size_t RingBuffer::peek(uint8_t *dest, size_t len) const {
  size_t avail = available();
  len = std::min(len, avail);
  if (len == 0)
    return 0;

  size_t t = tail.load(std::memory_order_acquire);
  size_t to_end = capacity - t;

  if (len <= to_end) {
    std::memcpy(dest, buffer + t, len);
  } else {
    std::memcpy(dest, buffer + t, to_end);
    std::memcpy(dest + to_end, buffer, len - to_end);
  }
  return len;
}

// 计算缓存区可用数据大小并返回
size_t RingBuffer::available() const {
  size_t h = head.load(std::memory_order_acquire);
  size_t t = tail.load(std::memory_order_acquire);
  return (h - t) & mask;
}

// 返回缓冲区的剩余空间
size_t RingBuffer::free_space() const {
  return capacity - available() - 1; // 留1字节区分满和空
}

// 清空缓冲区
void RingBuffer::clear() {
  tail.store(head.load(std::memory_order_relaxed), std::memory_order_release);
}

// 判断缓冲区是否为空
bool RingBuffer::empty() const {
  return head.load(std::memory_order_acquire) ==
         tail.load(std::memory_order_acquire);
}

// 判断缓冲区是否已满
bool RingBuffer::full() const {
  return ((head.load(std::memory_order_acquire) + 1) & mask) ==
         tail.load(std::memory_order_acquire);
}
} // namespace serial_hw