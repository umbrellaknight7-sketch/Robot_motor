#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace erob {
// Single producer / single consumer; each slot is allocated before threads
// start. Command queue preserves ordering and explicitly reports overflow.
template <class T, size_t N = 32> class SpscQueue {
  std::array<T, N> slots_;

  // Keep producer and consumer indices on separate cache lines.
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};

public:
  explicit SpscQueue(const T &initial) {
    for (auto &s : slots_)
      s = initial;
  }

  bool push(const T &v) noexcept {
    auto h = head_.load(std::memory_order_relaxed);
    auto next = (h + 1) % N;
    if (next == tail_.load(std::memory_order_acquire))
      return false;
    slots_[h] = v;
    head_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T &v) noexcept {
    auto t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire))
      return false;
    v = slots_[t];
    tail_.store((t + 1) % N, std::memory_order_release);
    return true;
  }
};

// Latest coherent state, SPSC triple buffer: slow readers never stall the bus.
template <class T> class Latest {
  std::array<T, 3> slots_;

  // Low two bits select the slot; bit 2 marks an unread publication.
  std::atomic<unsigned> middle_{1};
  unsigned front_ = 0, back_ = 2;

public:
  explicit Latest(const T &initial) {
    for (auto &s : slots_)
      s = initial;
  }

  void publish(const T &v) noexcept {
    slots_[back_] = v;
    back_ = middle_.exchange(back_ | 4, std::memory_order_acq_rel) & 3;
  }

  bool read(T &v) noexcept {
    const bool changed = (middle_.load(std::memory_order_acquire) & 4) != 0;
    if (changed)
      front_ = middle_.exchange(front_, std::memory_order_acq_rel) & 3;
    v = slots_[front_];
    return changed;
  }
};
} // namespace erob
