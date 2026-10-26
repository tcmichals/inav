/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dedicated Linux Quad-Core SBC Hardware Entry Point
 *
 * Targeted at Linux Single-Board Computers (e.g. Raspberry Pi 5, Jetson Orin) with PCIe BAR hardware bridge.
 * Pins real-time flight thread to Core 3 (SCHED_FIFO 99) and I/O reactor to Core 2.
 */

#include "flight_engine_template.hpp"
#include "config_registry.hpp"
#include "msp_server.hpp"
#include "posix_tcp_transport.hpp"
#include "blackbox_logger.hpp"
#include "linux_rt_hardener.hpp"
#include "icm42688p.hpp"
#include "dps310.hpp"
#include "gps_driver.hpp"

#include "abstractx/reactor/epoll_reactor.hpp"
#include "abstractx/reactor/async_socket.hpp"


#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <poll.h>
#include <fcntl.h>

using namespace abstractx;

static std::atomic<bool> g_system_running{true};
static SpscTlpRing<64> g_telemetry_ring;
static SpscTlpRing<64> g_logging_ring;

static drivers::gps::GpsDriver g_gps_driver;
static msp::MspServer<msp::PosixTcpTransport> g_msp_server{5760};
static logging::BlackboxLogger g_blackbox_logger;
static msp::MspLiveState g_msp_live_state;

// Minimal Dummy SBC Target for FlightEngine concept
struct LinuxSbcTarget {
    std::string_view name() const noexcept { return "LINUX_SBC"; }
    bool init() noexcept { return true; }
};

static void signal_handler(int /*sig*/) {
    g_system_running.store(false, std::memory_order_relaxed);
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Harden Real-Time Flight Loop on Core 3 (Priority 99, SCHED_FIFO, mlockall)
    target::sitl::LinuxRtHardener::harden_single_realtime_core(3);
    target::sitl::LinuxRtHardener::lock_process_memory();

    // 1. Config Registry Load

    ConfigRegistry::load_from_file("config.bin");

    // 2. Start MSP TCP Server (TCP 5760)
    g_msp_server.start();

    // 3. Start Blackbox Logger Thread (Core 0)
    g_blackbox_logger.start(g_logging_ring, "blackbox");

    // 4. Start Hardware TLP Telemetry Ingestion Receiver (VirtIO RPMsg /dev/rpmsg0 + UDP 9870)
    std::thread tlp_receiver([&]() {
        int rpmsg_fd = ::open("/dev/rpmsg0", O_RDWR | O_NONBLOCK);
        if (rpmsg_fd >= 0) {
            const char hello[] = "helo";
            (void)::write(rpmsg_fd, hello, 4);
            std::cout << "[INAV SBC] Connected directly to XuanTie E906 VirtIO RPMsg (/dev/rpmsg0)\n";
        }

        int sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock >= 0) {
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(9870);
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
        }

        std::array<pollfd, 2> pfds{};
        int nfds = 0;
        int rpmsg_idx = -1;
        int sock_idx = -1;

        if (rpmsg_fd >= 0) {
            rpmsg_idx = nfds;
            pfds[nfds].fd = rpmsg_fd;
            pfds[nfds].events = POLLIN;
            nfds++;
        }
        if (sock >= 0) {
            sock_idx = nfds;
            pfds[nfds].fd = sock;
            pfds[nfds].events = POLLIN;
            nfds++;
        }

        uint8_t buf[512];
        while (g_system_running.load(std::memory_order_relaxed)) {
            if (nfds == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            int ret = poll(pfds.data(), static_cast<nfds_t>(nfds), 50);
            if (ret <= 0) continue;

            // 1. Direct VirtIO RPMsg packets from XuanTie E906 Co-Processor
            if (rpmsg_idx >= 0 && (pfds[rpmsg_idx].revents & POLLIN)) {
                ssize_t n = ::read(rpmsg_fd, buf, sizeof(buf));
                if (n > 0 && (n % 64 == 0)) {
                    for (ssize_t off = 0; off < n; off += 64) {
                        Tlp64 tlp{};
                        std::memcpy(&tlp.wire, &buf[off], sizeof(asp_tlp64_t));
                        (void)g_telemetry_ring.push(tlp);
                    }
                }
            }

            // 2. UDP 9870 fallback packets (SITL simulation / bench)
            if (sock_idx >= 0 && (pfds[sock_idx].revents & POLLIN)) {
                ssize_t n = recv(sock, buf, sizeof(buf), 0);
                if (n == static_cast<ssize_t>(sizeof(asp_tlp64_t))) {
                    Tlp64 tlp{};
                    std::memcpy(&tlp.wire, buf, sizeof(asp_tlp64_t));
                    (void)g_telemetry_ring.push(tlp);
                }
            }
        }

        if (rpmsg_fd >= 0) ::close(rpmsg_fd);
        if (sock >= 0) ::close(sock);
    });

    std::cout << "Linux SBC Flight Engine Running (PCIe BAR + SCHED_FIFO Core 3).\n";

    LinuxSbcTarget sbc_target{};
    flight::FlightEngine<
        LinuxSbcTarget,
        drivers::imu::Icm42688P,
        drivers::baro::Dps310,
        drivers::gps::GpsDriver,
        4
    > sbc_engine{sbc_target, g_gps_driver};

    auto engine_task = sbc_engine.run_loop(g_telemetry_ring, g_logging_ring);
    
    while (g_system_running.load(std::memory_order_relaxed)) {
        engine_task.resume();

        // Feed MSP live state each flight loop cycle
        const auto& ahrs_angles = sbc_engine.ahrs_imu().angles();
        const auto& pos_state = sbc_engine.pos_estimator().state();
        const auto& gps_sample = g_gps_driver.latest_sample();

        g_msp_live_state.roll_decideg  = ahrs_angles.roll_decideg;
        g_msp_live_state.pitch_decideg = ahrs_angles.pitch_decideg;
        g_msp_live_state.yaw_deg       = static_cast<int16_t>(ahrs_angles.yaw_deg);
        g_msp_live_state.altitude_cm   = static_cast<int32_t>(-pos_state.pos_d_m * 100.0f);
        g_msp_live_state.vario_cms     = static_cast<int16_t>(-pos_state.vel_d_m_s * 100.0f);
        g_msp_live_state.cycle_time_us = 1000;
        g_msp_live_state.sensor_flags  = 0x31; // ACC | BARO | GPS

        g_msp_live_state.gps_lat       = gps_sample.latitude_1e7;
        g_msp_live_state.gps_lon       = gps_sample.longitude_1e7;
        g_msp_live_state.gps_alt_m     = static_cast<int16_t>(gps_sample.altitude_cm / 100);
        g_msp_live_state.gps_speed_cms = static_cast<uint16_t>(gps_sample.vel_n_cms);
        g_msp_live_state.gps_ground_course = gps_sample.ground_course_decideg;
        g_msp_live_state.gps_num_sats  = gps_sample.num_sats;
        g_msp_live_state.gps_fix       = (gps_sample.fix_type != drivers::gps::GpsFixType::NoFix) ? 1 : 0;

        g_msp_server.update_live_state(g_msp_live_state);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    sbc_engine.stop();
    g_msp_server.stop();
    g_blackbox_logger.stop();
    if (tlp_receiver.joinable()) {
        tlp_receiver.join();
    }

    std::cout << "Linux SBC Flight Engine Stopped Cleanly.\n";
    return 0;
}
