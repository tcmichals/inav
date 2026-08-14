/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Zero-Allocation C++20 Coroutine Task Engine & Sensor Awaiters
 */

#ifndef COROUTINE_TASK_HPP
#define COROUTINE_TASK_HPP

#include <coroutine>
#include <cstdint>
#include <cstddef>
#include <utility>
#include <optional>
#include <tuple>


namespace abstractx {

// Zero-Allocation Coroutine Static Memory Pool with FreeList Recycling
template <size_t PoolSize = 16384>
class CoroutineStaticPool {
public:
    static void* allocate(size_t size) noexcept {
        if (free_list_head_) {
            Node* node = free_list_head_;
            free_list_head_ = free_list_head_->next;
            return static_cast<void*>(node);
        }
        if (offset_ + size > PoolSize) {
            offset_ = 0; // Wrap around static pool
        }
        void* ptr = &pool_[offset_];
        offset_ += size;
        return ptr;
    }

    static void deallocate(void* ptr) noexcept {
        if (!ptr) return;
        Node* node = static_cast<Node*>(ptr);
        node->next = free_list_head_;
        free_list_head_ = node;
    }

    static void reset() noexcept {
        offset_ = 0;
        free_list_head_ = nullptr;
    }

private:
    struct Node {
        Node* next;
    };
    alignas(16) static inline uint8_t pool_[PoolSize];
    static inline size_t offset_{0};
    static inline Node* free_list_head_{nullptr};
};

template <typename T = void>
class Task {
public:
    struct promise_type {
        Task get_return_object() noexcept {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        static Task get_return_object_on_allocation_failure() noexcept {
            return Task{nullptr};
        }
        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }

        void unhandled_exception() noexcept {}
        void return_void() noexcept {}

        // Overload operator new to allocate from static memory pool (Zero Dynamic Heap!)
        static void* operator new(size_t size) noexcept {
            return CoroutineStaticPool<16384>::allocate(size);
        }

        static void operator delete(void* ptr) noexcept {
            CoroutineStaticPool<16384>::deallocate(ptr);
        }
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

// C++20 Custom Awaiter: Yield control for one tick
struct YieldTick {
    constexpr bool await_ready() const noexcept { return false; }
    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};

// C++20 Custom Awaiter: IMU Burst Telemetry Awaiter
template <typename RingType>
struct ImuSampleAwaiter {
    RingType& ring;
    typename RingType::value_type result{};

    bool await_ready() noexcept {
        if (!ring.empty()) {
            auto opt = ring.pop();
            if (opt.has_value()) {
                result = opt.value();
                return true;
            }
        }
        return false;
    }

    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}

    typename RingType::value_type await_resume() noexcept {
        return result;
    }
};

// C++20 Custom Awaiter: Barometer Sensor Telemetry Awaiter
template <typename BaroDriverType, typename TlpType>
struct BaroSampleAwaiter {
    const TlpType& tlp;
    bool ready{false};

    bool await_ready() noexcept {
        return ready;
    }

    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}

    auto await_resume() noexcept {
        return BaroDriverType::parse_tlp(tlp);
    }
};

// C++20 Custom Awaiter: GPS Sensor Telemetry Awaiter
template <typename GpsDriverType>
struct GpsSampleAwaiter {
    const GpsDriverType& driver;

    bool await_ready() noexcept {
        return driver.latest_sample().valid;
    }

    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}

    auto await_resume() noexcept {
        return driver.latest_sample();
    }
};

// C++20 Zero-Allocation Multi-Task Parallel Combinator: WhenAll
template <typename... Tasks>
struct WhenAllAwaiter {
    std::tuple<Tasks&...> tasks;

    constexpr explicit WhenAllAwaiter(Tasks&... t) noexcept : tasks(t...) {}

    bool await_ready() noexcept {
        std::apply([](auto&... t) {
            ((!t.done() ? (t.resume(), 0) : 0), ...);
        }, tasks);

        return std::apply([](auto&... t) {
            return (t.done() && ...);
        }, tasks);
    }

    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};

template <typename... Tasks>
[[nodiscard]] constexpr auto when_all(Tasks&... tasks) noexcept {
    return WhenAllAwaiter<Tasks...>{tasks...};
}

// C++20 Zero-Allocation Multi-Task Parallel Combinator: WhenAny
template <typename... Tasks>
struct WhenAnyAwaiter {
    std::tuple<Tasks&...> tasks;

    constexpr explicit WhenAnyAwaiter(Tasks&... t) noexcept : tasks(t...) {}

    bool await_ready() noexcept {
        std::apply([](auto&... t) {
            ((!t.done() ? (t.resume(), 0) : 0), ...);
        }, tasks);

        return std::apply([](auto&... t) {
            return (t.done() || ...);
        }, tasks);
    }

    constexpr void await_suspend(std::coroutine_handle<>) const noexcept {}
    constexpr void await_resume() const noexcept {}
};

template <typename... Tasks>
[[nodiscard]] constexpr auto when_any(Tasks&... tasks) noexcept {
    return WhenAnyAwaiter<Tasks...>{tasks...};
}


} // namespace abstractx

#endif // COROUTINE_TASK_HPP

