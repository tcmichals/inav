/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Lock-Free Background Disk Blackbox Logging Thread (Core 0, SCHED_OTHER)
 */

#ifndef BLACKBOX_LOGGER_HPP
#define BLACKBOX_LOGGER_HPP

#include "spsc_tlp_ring.hpp"
#include "asp_tlp64.hpp"
#include <cstdint>
#include <thread>
#include <atomic>
#include <fstream>

namespace abstractx::logging {

class BlackboxLogger {
public:
    // Starts background disk logger thread on non-RT Core 0 to prevent SD card stalls from affecting flight loop
    static void start_background_disk_writer(SpscTlpRing<64>& log_ring, const char* filepath, std::atomic<bool>& running) noexcept {
        std::thread([&log_ring, filepath, &running]() {
            std::ofstream disk_file(filepath, std::ios::binary | std::ios::app);
            if (!disk_file.is_open()) return;

            while (running.load(std::memory_order_relaxed)) {
                if (!log_ring.empty()) {
                    auto opt_tlp = log_ring.pop();
                    if (opt_tlp.has_value()) {
                        Tlp64 tlp = opt_tlp.value();
                        disk_file.write(reinterpret_cast<const char*>(&tlp), sizeof(Tlp64));
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
            }
            disk_file.flush();
        }).detach();
    }
};

} // namespace abstractx::logging

#endif // BLACKBOX_LOGGER_HPP
