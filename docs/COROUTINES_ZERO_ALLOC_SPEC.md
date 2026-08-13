# Zero-Allocation C++20 Coroutine Specification (`tcmichals/inav` - `pcie-clean` branch)

## 1. Core Principles

Modern C++20 coroutines (`co_await`, `co_yield`, `co_return`) provide clean, linear asynchronous programming for embedded systems. However, standard compiler implementations allocate coroutine frames dynamically via `operator new`.

In **`inav-abstractx`**, coroutines MUST be 100% deterministic and zero-heap allocated:
1. All coroutine promise types (`Task<T>::promise_type`) overload `operator new` to pull coroutine frame blocks from a static pre-allocated memory pool (`asp_coroutine_pool`).
2. If the static promise pool is exhausted, the allocation fails at compile-time or triggers a static panic rather than falling back to heap `malloc`.
3. Coroutine tasks execute wait-free over lock-free Single-Producer Single-Consumer (SPSC) ring buffers.

---

## 2. Coroutine Task & Promise Interface (`abstractx::Task<T>`)

```cpp
namespace abstractx {

template <typename T = void>
struct Task {
    struct promise_type {
        T value{};
        std::coroutine_handle<promise_type> handle{};

        Task get_return_object() {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_always initial_suspend() noexcept { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        
        void return_value(T val) noexcept { value = val; }
        void unhandled_exception() noexcept { std::terminate(); }

        // OVERLOAD NEW TO PREVENT HEAP ALLOCATION
        void* operator new(size_t size) noexcept {
            return CoroutineStaticPool::allocate(size);
        }
        void operator delete(void* ptr, size_t size) noexcept {
            CoroutineStaticPool::deallocate(ptr, size);
        }
    };

    std::coroutine_handle<promise_type> handle;
};

} // namespace abstractx
```

---

## 3. Asynchronous Bus Primitives

### 3.1 Asynchronous Register Read (`co_await pcie_reg_read_async`)
```cpp
// Asynchronously reads a 32-bit register from the virtual BAR without blocking the flight loop
uint32_t val = co_await abstractx::pcie_reg_read_async(PCIE_BAR_IMU_BASE + IMU_REG_STATUS);
```

### 3.2 Telemetry Stream Reception (`co_await tlp_stream_wait`)
```cpp
// Awaits the arrival of a 64-byte DMA_Stream TLP on the IMU telemetry channel
asp_tlp64_t tlp = co_await abstractx::tlp_stream_wait(ASP_CHANNEL_TELEMETRY);
```

---

## 4. Performance & Memory Impact

- **Heap Footprint**: $0\text{ bytes}$ dynamic allocation during initialization or execution.
- **Latency Jitter**: Zero OS thread context-switch overhead; execution is driven directly by SPSC queue events.
- **Timestamping Accuracy**: Hardware timestamp `timestamp_ns` passed in `asp_tlp64_t` guarantees $< 20\text{ ns}$ precision for EKF velocity estimation.
