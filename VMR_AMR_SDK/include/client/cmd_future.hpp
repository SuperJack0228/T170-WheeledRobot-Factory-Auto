#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>

// 带取消功能的 future 包装器
template <typename T>
struct CmdFuture {
  uint64_t id;
  std::future<T> future;
  std::function<void()> cancel;

  template <class Rep, class Per>
  std::future_status wait_for(const std::chrono::duration<Rep, Per>& rel_time) {
    return future.wait_for(rel_time);
  }

  T get() { return future.get(); }
};