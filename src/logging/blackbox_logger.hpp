/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2015-2026 Cleanflight / Betaflight / INAV Contributors (Nicholas Sherlock, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Lock-Free Background Disk Blackbox Logger (Linux SITL Only)
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/blackbox/blackbox.c
 *   - Upstream INAV: src/main/blackbox/blackbox.c
 *
 * Drains SPSC ring on a background thread (Core 0, SCHED_OTHER).
 * Joinable thread with proper shutdown. Periodic flush + file rotation.
 */

#ifndef BLACKBOX_LOGGER_HPP
#define BLACKBOX_LOGGER_HPP


#include "spsc_tlp_ring.hpp"
#include "asp_tlp64.hpp"
#include <cstdint>
#include <cstdio>

#if defined(__linux__) || defined(__APPLE__)
#include <thread>
#include <atomic>
#include <fstream>
#include <string>

namespace abstractx::logging {

class BlackboxLogger {
public:
    static constexpr size_t MAX_FILE_SIZE = 10 * 1024 * 1024; // 10MB per log file
    static constexpr size_t FLUSH_INTERVAL_TLPS = 64;          // Flush every 64 TLPs

    BlackboxLogger() noexcept = default;
    ~BlackboxLogger() { stop(); }

    // Non-copyable, non-movable
    BlackboxLogger(const BlackboxLogger&) = delete;
    BlackboxLogger& operator=(const BlackboxLogger&) = delete;

    void start(SpscTlpRing<64>& log_ring, const char* base_path) noexcept {
        if (running_.load(std::memory_order_relaxed)) return;

        running_.store(true, std::memory_order_relaxed);
        base_path_ = base_path;

        worker_ = std::thread([this, &log_ring]() {
            run(log_ring);
        });
    }

    void stop() noexcept {
        running_.store(false, std::memory_order_relaxed);
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool is_running() const noexcept {
        return running_.load(std::memory_order_relaxed);
    }

    uint64_t total_tlps_written() const noexcept {
        return total_tlps_.load(std::memory_order_relaxed);
    }

private:
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> total_tlps_{0};
    std::thread worker_;
    std::string base_path_;
    uint32_t file_index_{1};

    std::string make_filename() const {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s_%04u.bin", base_path_.c_str(), file_index_);
        return std::string(buf);
    }

    void run(SpscTlpRing<64>& log_ring) noexcept {
        std::ofstream file(make_filename(), std::ios::binary | std::ios::app);
        if (!file.is_open()) return;

        size_t current_file_size = 0;
        size_t tlps_since_flush = 0;

        while (running_.load(std::memory_order_relaxed)) {
            bool did_work = false;

            while (!log_ring.empty()) {
                auto opt_tlp = log_ring.pop();
                if (!opt_tlp.has_value()) break;

                Tlp64 tlp = opt_tlp.value();
                file.write(reinterpret_cast<const char*>(&tlp), sizeof(Tlp64));
                current_file_size += sizeof(Tlp64);
                total_tlps_.fetch_add(1, std::memory_order_relaxed);
                tlps_since_flush++;
                did_work = true;

                // Periodic flush
                if (tlps_since_flush >= FLUSH_INTERVAL_TLPS) {
                    file.flush();
                    tlps_since_flush = 0;
                }

                // File rotation
                if (current_file_size >= MAX_FILE_SIZE) {
                    file.flush();
                    file.close();
                    file_index_++;
                    file.open(make_filename(), std::ios::binary | std::ios::app);
                    if (!file.is_open()) return;
                    current_file_size = 0;
                }
            }

            if (!did_work) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }

        file.flush();
    }
};

} // namespace abstractx::logging

#else // Non-Linux (Pico 2 W) — stub

namespace abstractx::logging {

class BlackboxLogger {
public:
    void start(auto& /*ring*/, const char* /*path*/) noexcept {}
    void stop() noexcept {}
    bool is_running() const noexcept { return false; }
    uint64_t total_tlps_written() const noexcept { return 0; }
};

} // namespace abstractx::logging

#endif // __linux__ || __APPLE__

#endif // BLACKBOX_LOGGER_HPP
