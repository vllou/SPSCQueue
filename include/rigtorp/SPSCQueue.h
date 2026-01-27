/*
Copyright (c) 2020 Erik Rigtorp <erik@rigtorp.se>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#pragma once

#include <atomic>
#include <cassert>
#include <cstddef>
#include <memory> // std::allocator
#include <new>    // std::hardware_destructive_interference_size
#include <stdexcept>
#include <type_traits> // std::enable_if, std::is_*_constructible

#ifdef __has_cpp_attribute
#if __has_cpp_attribute(nodiscard)
#define RIGTORP_NODISCARD [[nodiscard]]
#endif
#endif
#ifndef RIGTORP_NODISCARD
#define RIGTORP_NODISCARD
#endif

namespace rigtorp {

    // STL 容器风格 使用std::allocator 分配内存
template <typename T, typename Allocator = std::allocator<T>> 
class SPSCQueue {

    // allocate_at_least 方法返回一个结构体，包含指针和实际分配的元素数量
    // 因为 allocate_at_least 会分配至少指定数量的元素，但可能会多分配一些
#if defined(__cpp_if_constexpr) && defined(__cpp_lib_void_t)
  template <typename Alloc2, typename = void>
  struct has_allocate_at_least : std::false_type {};

  template <typename Alloc2>
  struct has_allocate_at_least<
      Alloc2, std::void_t<typename Alloc2::value_type,
                          decltype(std::declval<Alloc2 &>().allocate_at_least(
                              size_t{}))>> : std::true_type {};
#endif

public:
  explicit SPSCQueue(const size_t capacity,
                     const Allocator &allocator = Allocator())
      : capacity_(capacity), allocator_(allocator) {
    // The queue needs at least one element
    if (capacity_ < 1) {
      capacity_ = 1;
    }
    // 循环队列，所以容量为队列最大容量-1
    capacity_++; // Needs one slack element
    // Prevent overflowing size_t
    // 防止传入的队列数量溢出 size_t 的最大值
    // 为了让队列都处于一个隔离范围，所以需要前后都有一个 kPadding数量T大小 的空间
    if (capacity_ > SIZE_MAX - 2 * kPadding) {
      capacity_ = SIZE_MAX - 2 * kPadding;
    }

    // 使用分配分配分配内存
#if defined(__cpp_if_constexpr) && defined(__cpp_lib_void_t)
    if constexpr (has_allocate_at_least<Allocator>::value) {
      auto res = allocator_.allocate_at_least(capacity_ + 2 * kPadding);
      slots_ = res.ptr;  // 首指针
      capacity_ = res.count - 2 * kPadding; // 实际分配的元素数量-前后padding
    } else {
      slots_ = std::allocator_traits<Allocator>::allocate(
          allocator_, capacity_ + 2 * kPadding);
    }
#else
    slots_ = std::allocator_traits<Allocator>::allocate(
        allocator_, capacity_ + 2 * kPadding);
#endif

    static_assert(alignof(SPSCQueue<T>) == kCacheLineSize, "");  // 对齐缓存行大小
    static_assert(sizeof(SPSCQueue<T>) >= 3 * kCacheLineSize, ""); // 当前类容量至少3个缓存行大小

    //运行时检查：readIdx_ 和 writeIdx_ 的内存地址距离 ≥ 64 字节
    assert(reinterpret_cast<char *>(&readIdx_) - reinterpret_cast<char *>(&writeIdx_) >= static_cast<std::ptrdiff_t>(kCacheLineSize));
  }

  ~SPSCQueue() {
    while (front()) {
      pop();
    }
    std::allocator_traits<Allocator>::deallocate(allocator_, slots_,
                                                 capacity_ + 2 * kPadding);
  }

  // non-copyable and non-movable
  SPSCQueue(const SPSCQueue &) = delete;
  SPSCQueue &operator=(const SPSCQueue &) = delete;

  // 入队
  template <typename... Args>
  void emplace(Args &&...args) noexcept(std::is_nothrow_constructible<T, Args &&...>::value) 
  {
    // 是否可以使用 Args&&... 构造 T 类型
    static_assert(std::is_constructible<T, Args &&...>::value, "T must be constructible with Args&&...");

    auto const writeIdx = writeIdx_.load(std::memory_order_relaxed);
    auto nextWriteIdx = writeIdx + 1;
    if (nextWriteIdx == capacity_) {
      nextWriteIdx = 0;
    }

    // 用 release 的线程，“发布”它之前做的事；
    // 用 acquire 的线程，“获取”并看到那些事。
    while (nextWriteIdx == readIdxCache_) {  // 满队列 刷新一下缓存
      readIdxCache_ = readIdx_.load(std::memory_order_acquire);  // 读， 在此之后的读写操作不会被重排序到此之前
    }
    new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);
    writeIdx_.store(nextWriteIdx, std::memory_order_release);  // 写， 在此之前的读写操作不会被重排序到此之后
  }

  template <typename... Args>
  RIGTORP_NODISCARD bool try_emplace(Args &&...args) noexcept(std::is_nothrow_constructible<T, Args &&...>::value) 
  {
    static_assert(std::is_constructible<T, Args &&...>::value, "T must be constructible with Args&&...");
    
    auto const writeIdx = writeIdx_.load(std::memory_order_relaxed);
    auto nextWriteIdx = writeIdx + 1;
    if (nextWriteIdx == capacity_) {
      nextWriteIdx = 0;
    }
    if (nextWriteIdx == readIdxCache_) {
      readIdxCache_ = readIdx_.load(std::memory_order_acquire);
      if (nextWriteIdx == readIdxCache_) {
        return false; // 如果队列满， 则返回 false
      }
    }

    new (&slots_[writeIdx + kPadding]) T(std::forward<Args>(args)...);
    writeIdx_.store(nextWriteIdx, std::memory_order_release);
    return true;
  }

  void push(const T &v) noexcept(std::is_nothrow_copy_constructible<T>::value) 
  {
    static_assert(std::is_copy_constructible<T>::value, "T must be copy constructible");
    emplace(v);
  }

  template <typename P, typename = typename std::enable_if<std::is_constructible<T, P &&>::value>::type>
  void push(P &&v) noexcept(std::is_nothrow_constructible<T, P &&>::value) 
  {
    emplace(std::forward<P>(v));
  }

  RIGTORP_NODISCARD bool
  try_push(const T &v) noexcept(std::is_nothrow_copy_constructible<T>::value) 
  {
    static_assert(std::is_copy_constructible<T>::value, "T must be copy constructible");
    return try_emplace(v);
  }

  template <typename P, typename = typename std::enable_if<std::is_constructible<T, P &&>::value>::type>
  RIGTORP_NODISCARD bool
  try_push(P &&v) noexcept(std::is_nothrow_constructible<T, P &&>::value) 
  {
    return try_emplace(std::forward<P>(v));
  }

  // 头指针
  RIGTORP_NODISCARD T* front() noexcept
  {
      auto const readIdx = readIdx_.load(std::memory_order_relaxed);
      if (readIdx == writeIdxCache_)
      {
          writeIdxCache_ = writeIdx_.load(std::memory_order_acquire);

          if (writeIdxCache_ == readIdx)
          {
              return nullptr;  // 当前空
          }
      }
      return &slots_[readIdx + kPadding];
  }

  void pop() noexcept
  {
      static_assert(std::is_nothrow_destructible<T>::value, "T must be nothrow destructible");

      auto const readIdx = readIdx_.load(std::memory_order_relaxed);
      assert(writeIdx_.load(std::memory_order_acquire) != readIdx && 
             "Can only call pop() after front() has returned a non-nullptr");

      slots_[readIdx + kPadding].~T();
      auto nextReadIdx = readIdx + 1;
      if (nextReadIdx == capacity_)
      {
          nextReadIdx = 0;
      }
      readIdx_.store(nextReadIdx, std::memory_order_release);
  }

  RIGTORP_NODISCARD size_t size() const noexcept {
    std::ptrdiff_t diff = writeIdx_.load(std::memory_order_acquire) -
                          readIdx_.load(std::memory_order_acquire);
    if (diff < 0) {
      diff += capacity_;
    }
    return static_cast<size_t>(diff);
  }

  RIGTORP_NODISCARD bool empty() const noexcept {
    return writeIdx_.load(std::memory_order_acquire) ==
           readIdx_.load(std::memory_order_acquire);
  }

  RIGTORP_NODISCARD size_t capacity() const noexcept { return capacity_ - 1; }

private:
#ifdef __cpp_lib_hardware_interference_size
  // CPU 的缓存行大小
  static constexpr size_t kCacheLineSize =
      std::hardware_destructive_interference_size;  
#else
  // CPU 的缓存行大小
  static constexpr size_t kCacheLineSize = 64;
#endif

  // Padding to avoid false sharing between slots_ and adjacent allocations
  // 向上取整， 确保T类型的数组可以占满整个cpu 缓存行
  static constexpr size_t kPadding = (kCacheLineSize - 1) / sizeof(T) + 1;

private:
  size_t capacity_;  // 容量
  T *slots_;  // 槽位
#if defined(__has_cpp_attribute) && __has_cpp_attribute(no_unique_address)
  Allocator allocator_ [[no_unique_address]];
#else
  Allocator allocator_;  // 分配器
#endif

  // Align to cache line size in order to avoid false sharing
  // readIdxCache_ and writeIdxCache_ is used to reduce the amount of cache
  // coherency traffic
  // 规定内存地址必须是缓存行大小的整数倍， 使其可以独占一个缓存行， 避免伪共享
  alignas(kCacheLineSize) std::atomic<size_t> writeIdx_ = {0};
  alignas(kCacheLineSize) size_t readIdxCache_ = 0;
  alignas(kCacheLineSize) std::atomic<size_t> readIdx_ = {0};
  alignas(kCacheLineSize) size_t writeIdxCache_ = 0;
};
} // namespace rigtorp
