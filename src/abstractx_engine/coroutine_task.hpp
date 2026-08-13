/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Zero-Allocation C++20 Coroutine Task Engine
 */

#ifndef COROUTINE_TASK_HPP
#define COROUTINE_TASK_HPP

#include <coroutine>
#include <cstdint>
#include <cstddef>
#include <utility>

namespace abstractx {

// Zero-Allocation Coroutine Static Memory Pool
template <size_t PoolSize = 4096>
class CoroutineStaticPool {
public:
    static void* allocate(size_t size) noexcept {
        if (offset_ + size > PoolSize) return nullptr;
        void* ptr = &pool_[offset_];
        offset_ += size;
        return ptr;
    }

    static void reset() noexcept { offset_ = 0; }

private:
    alignas(16) static inline uint8_t pool_[PoolSize];
    static inline size_t offset_{0};
};

template <typename T = void>
class Task {
public:
    struct promise_type {
        Task get_return_object() noexcept {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        void unhandled_exception() noexcept {}
        void return_void() noexcept {}

        // Overload operator new to allocate from static memory pool (Zero Dynamic Heap!)
        static void* operator new(size_t size) noexcept {
            return CoroutineStaticPool<4096>::allocate(size);
        }

        static void operator delete(void* /*ptr*/) noexcept {}
    };

    explicit Task(std::coroutine_handle<promise_type> handle) noexcept : handle_(handle) {}
    ~Task() noexcept {
        if (handle_) handle_.destroy();
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Task(Task&& other) noexcept : handle_(other.handle_) {
        other.handle_ = nullptr;
    }

    Task& operator=(Task&& other) noexcept {
        if (this != &other) {
            if (handle_) handle_.destroy();
            handle_ = other.handle_;
            other.handle_ = nullptr;
        }
        return *this;
    }

    bool done() const noexcept {
        return !handle_ || handle_.done();
    }

    bool resume() noexcept {
        if (handle_ && !handle_.done()) {
            handle_.resume();
            return !handle_.done();
        }
        return false;
    }

private:
    std::coroutine_handle<promise_type> handle_{nullptr};
};

} // namespace abstractx

#endif // COROUTINE_TASK_HPP
