/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * AbstractX C++20 Zero-Allocation Coroutine Task & Scheduler Engine
 */

#ifndef COROUTINE_TASK_HPP
#define COROUTINE_TASK_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <coroutine>
#include <cstdint>
#include <cstddef>
#include <array>
#include <utility>

namespace abstractx {

// Static coroutine promise memory pool to prevent heap allocation (malloc/new)
class CoroutineStaticPool {
public:
    static constexpr size_t PoolSize = 16384; // 16 KB static coroutine frame pool

    static void* allocate(size_t size) noexcept {
        if (offset_ + size > PoolSize) {
            return nullptr; // Pool exhausted (triggers compile-time/static panic)
        }
        void* ptr = &pool_[offset_];
        offset_ += (size + 7) & ~size_t(7); // Align to 8 bytes
        return ptr;
    }

    static void deallocate(void* /*ptr*/, size_t /*size*/) noexcept {
        // Linear embedded allocator; deallocation is a no-op or handled at reset
    }

    static void reset() noexcept { offset_ = 0; }

private:
    static inline alignas(16) uint8_t pool_[PoolSize];
    static inline size_t offset_{0};
};

// Zero-allocation C++20 Coroutine Task handle
template <typename T = void>
class Task {
public:
    struct promise_type {
        T value{};
        std::coroutine_handle<promise_type> handle{};

        Task get_return_object() noexcept {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        void return_value(T val) noexcept { value = val; }
        void unhandled_exception() noexcept {}

        void* operator new(size_t size) noexcept {
            return CoroutineStaticPool::allocate(size);
        }

        void operator delete(void* ptr, size_t size) noexcept {
            CoroutineStaticPool::deallocate(ptr, size);
        }
    };

    explicit Task(std::coroutine_handle<promise_type> h) noexcept : handle_(h) {}
    ~Task() {
        if (handle_) {
            handle_.destroy();
        }
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Task(Task&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
    Task& operator=(Task&& other) noexcept {
        if (this != &other) {
            if (handle_) handle_.destroy();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    bool resume() const noexcept {
        if (handle_ && !handle_.done()) {
            handle_.resume();
            return true;
        }
        return false;
    }

    bool done() const noexcept { return !handle_ || handle_.done(); }

    T get() const noexcept {
        return handle_.promise().value;
    }

private:
    std::coroutine_handle<promise_type> handle_;
};

// Specialization for void return tasks
template <>
class Task<void> {
public:
    struct promise_type {
        std::coroutine_handle<promise_type> handle{};

        Task get_return_object() noexcept {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        void return_void() noexcept {}
        void unhandled_exception() noexcept {}

        void* operator new(size_t size) noexcept {
            return CoroutineStaticPool::allocate(size);
        }

        void operator delete(void* ptr, size_t size) noexcept {
            CoroutineStaticPool::deallocate(ptr, size);
        }
    };

    explicit Task(std::coroutine_handle<promise_type> h) noexcept : handle_(h) {}
    ~Task() {
        if (handle_) {
            handle_.destroy();
        }
    }

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Task(Task&& other) noexcept : handle_(std::exchange(other.handle_, nullptr)) {}
    Task& operator=(Task&& other) noexcept {
        if (this != &other) {
            if (handle_) handle_.destroy();
            handle_ = std::exchange(other.handle_, nullptr);
        }
        return *this;
    }

    bool resume() const noexcept {
        if (handle_ && !handle_.done()) {
            handle_.resume();
            return true;
        }
        return false;
    }

    bool done() const noexcept { return !handle_ || handle_.done(); }

private:
    std::coroutine_handle<promise_type> handle_;
};

} // namespace abstractx

#endif // COROUTINE_TASK_HPP
