/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - C++20 Concept-Constrained Target Platform Interface (Zero #ifdefs!)
 */

#ifndef TARGET_INTERFACE_HPP
#define TARGET_INTERFACE_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <concepts>
#include <cstdint>
#include <string_view>

namespace abstractx::target {

// C++20 Concept enforcing target platform interface requirements at compile-time (NO #ifdefs!)
namespace concepts {

template <typename T>
concept IsPlatform = requires(T platform, uint32_t addr, uint32_t val, SpscTlpRing<64>& ring) {
    { platform.name() } -> std::same_as<std::string_view>;
    { platform.init() } -> std::same_as<bool>;
    { platform.reg_read32(addr) } -> std::same_as<uint32_t>;
    { platform.reg_write32(addr, val) } -> std::same_as<void>;
    { platform.process_telemetry(ring) } -> std::same_as<void>;
};

} // namespace concepts

// Compile-Time Platform Dispatcher (Zero preprocessor #ifdef hell!)
template <typename Platform>
class TargetAdapter {
    static_assert(concepts::IsPlatform<Platform>, "Platform type MUST satisfy target::concepts::IsPlatform");

public:
    explicit TargetAdapter(Platform impl) : impl_(impl) {}

    bool start() {
        return impl_.init();
    }

    uint32_t read(uint32_t addr) {
        return impl_.reg_read32(addr);
    }

    void write(uint32_t addr, uint32_t val) {
        impl_.reg_write32(addr, val);
    }

    void poll(SpscTlpRing<64>& ring) {
        impl_.process_telemetry(ring);
    }

    std::string_view name() const { return impl_.name(); }

private:
    Platform impl_;
};

} // namespace abstractx::target

#endif // TARGET_INTERFACE_HPP
