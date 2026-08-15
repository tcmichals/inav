/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Linux SBC Hardware FPGA PCIe / UIO DMA Offload Transport
 *
 * Architecture:
 *   - Control Plane (Linux -> FPGA): Writes 64B TLP command/config packets to FPGA Control BAR (0x0000..0x7FFF).
 *   - Data Plane (FPGA -> Linux): FPGA SPI state machine reads IMU on DRDY edge, packs 64B TLP,
 *     and streams via PCIe/AXI DMA directly into host Linux SPSC shared-memory ring buffer.
 *   - Interrupts: Handled via Linux UIO eventfd or VFIO MSI-X epoll reactor.
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef LINUX_FPGA_TRANSPORT_HPP
#define LINUX_FPGA_TRANSPORT_HPP

#include "asp_tlp64.hpp"
#include "pcie_bar_map.hpp"
#include "spsc_tlp_ring.hpp"
#include "abstractx_engine/coroutine_task.hpp"
#include <cstdint>
#include <cstring>
#include <atomic>
#include <span>

namespace abstractx::target::linux_io {

struct FpgaDmaRingHeader {
    std::atomic<uint32_t> head{0u}; // Written by FPGA DMA engine
    std::atomic<uint32_t> tail{0u}; // Written by Linux software consumer
    uint32_t              capacity{64u};
    uint32_t              reserved{0u};
};

class LinuxFpgaTransport {
public:
    explicit LinuxFpgaTransport(volatile uint8_t* bar0_base,
                                volatile FpgaDmaRingHeader* dma_hdr,
                                volatile Tlp64* dma_slots,
                                int uio_event_fd = -1) noexcept
        : bar0_base_{bar0_base}, dma_hdr_{dma_hdr}, dma_slots_{dma_slots}, uio_event_fd_{uio_event_fd} {}

    // -------------------------------------------------------------------------
    // Control Plane: Linux -> FPGA Register / Config TLP Write
    // -------------------------------------------------------------------------
    [[nodiscard]] bool send_tlp(const Tlp64& tlp) noexcept {
        if (bar0_base_ == nullptr) {
            return false;
        }

        // Virtual BAR address maps directly to FPGA PCIe BAR offset
        const uint32_t bar_offset = tlp.wire.target_address;
        if (bar_offset + sizeof(Tlp64) > 0x10000u) { // 64KB BAR0 space
            return false;
        }

        // 64-byte single-cycle or 512-bit burst write to FPGA Mailbox
        volatile uint8_t* dest = bar0_base_ + bar_offset;
        const uint8_t* src = reinterpret_cast<const uint8_t*>(&tlp);

        for (size_t i = 0u; i < sizeof(Tlp64); ++i) {
            dest[i] = src[i];
        }

        #if defined(__x86_64__) || defined(__aarch64__) || defined(__riscv)
        // Memory barrier to ensure FPGA PCIe write commit
        std::atomic_thread_fence(std::memory_order_release);
        #endif

        return true;
    }

    // -------------------------------------------------------------------------
    // Data Plane: Pull incoming TLPs streamed by FPGA DMA into software SPSC ring
    // -------------------------------------------------------------------------
    [[nodiscard]] uint32_t poll_dma_stream(SpscTlpRing<64u>& ring) noexcept {
        if (dma_hdr_ == nullptr || dma_slots_ == nullptr) {
            return 0u;
        }

        uint32_t head = dma_hdr_->head.load(std::memory_order_acquire);
        uint32_t tail = dma_hdr_->tail.load(std::memory_order_relaxed);
        uint32_t count = 0u;

        while (tail != head) {
            const uint32_t slot_idx = tail % dma_hdr_->capacity;
            
            // Read 64B TLP written by FPGA hardware DMA
            Tlp64 tlp{};
            const volatile uint8_t* src = reinterpret_cast<const volatile uint8_t*>(&dma_slots_[slot_idx]);
            uint8_t* dst = reinterpret_cast<uint8_t*>(&tlp);
            
            for (size_t i = 0u; i < sizeof(Tlp64); ++i) {
                dst[i] = src[i];
            }

            // Push into Top-Half SPSC ring buffer
            if (ring.push(tlp)) {
                count++;
                tail++;
            } else {
                break; // Software ring full
            }
        }

        dma_hdr_->tail.store(tail, std::memory_order_release);
        return count;
    }

    // -------------------------------------------------------------------------
    // Coroutine Async Poller (Yields when waiting for FPGA DMA IRQ / eventfd)
    // -------------------------------------------------------------------------
    Task<void> run_fpga_dma_service(SpscTlpRing<64u>& ring, std::atomic<bool>& running) noexcept {
        while (running.load(std::memory_order_relaxed)) {
            // Poll any immediate DMA frames
            uint32_t received = poll_dma_stream(ring);
            if (received == 0u) {
                // Yield coroutine until next hardware event / tick (e.g. 50us)
                co_await sleep_us(50u);
            }
        }
        co_return;
    }

    [[nodiscard]] bool is_connected() const noexcept {
        return (bar0_base_ != nullptr && dma_hdr_ != nullptr);
    }

private:
    volatile uint8_t*           bar0_base_{nullptr};
    volatile FpgaDmaRingHeader* dma_hdr_{nullptr};
    volatile Tlp64*             dma_slots_{nullptr};
    int                         uio_event_fd_{-1};
};

} // namespace abstractx::target::linux_io

#endif // LINUX_FPGA_TRANSPORT_HPP
