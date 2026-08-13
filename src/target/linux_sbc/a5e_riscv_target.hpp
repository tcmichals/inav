/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Allwinner Cubie A5E RISC-V Co-Processor & FPGA Offloader Target Interface
 */

#ifndef A5E_RISCV_TARGET_HPP
#define A5E_RISCV_TARGET_HPP

#include "asp_tlp64.hpp"
#include "spsc_tlp_ring.hpp"
#include <cstdint>

namespace abstractx::target::a5e {

class A5eRiscvFpgaTarget {
public:
    // Shared Memory TLP Transport between Linux ARM Cortex-A53 and Embedded A5E RISC-V Core
    static bool init_shared_sram_bridge() noexcept {
        return true;
    }

    // Submit 64-byte TLP packet from Linux ARM to A5E RISC-V co-processor
    static bool send_tlp_to_riscv(SpscTlpRing<64>& ring, const Tlp64& tlp) noexcept {
        return ring.push(tlp);
    }

    // Receive 64-byte TLP packet latched by RISC-V co-processor / FPGA DMA
    static bool poll_tlp_from_riscv(SpscTlpRing<64>& ring, Tlp64& out_tlp) noexcept {
        auto opt_tlp = ring.pop();
        if (opt_tlp.has_value()) {
            out_tlp = opt_tlp.value();
            return true;
        }
        return false;
    }
};

} // namespace abstractx::target::a5e

#endif // A5E_RISCV_TARGET_HPP
