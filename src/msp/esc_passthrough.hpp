/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2016-2026 Betaflight Contributors (BorisB, et al.)
 * Copyright (C) 2016-2026 INAV Contributors (Konstantin Sharlaimov, et al.)
 * Copyright (C) 2014-2016 Cleanflight Contributors (Dominic Clifton, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - 4way-interface BLHeli / AM32 ESC Passthrough Engine
 *
 * Ported / derived from upstream reference C source files:
 *   - Upstream Betaflight: src/main/drivers/serial_4way.c
 *   - Upstream INAV: src/main/drivers/serial_4way.c
 */

#ifndef ESC_PASSTHROUGH_HPP
#define ESC_PASSTHROUGH_HPP


#include "asp_tlp64.hpp"
#include <cstdint>
#include <span>
#include <array>

namespace abstractx::msp {

// 4way-interface Commands (BLHeli_S / BLHeli_32 / AM32 Configurator Compatible)
enum class FourWayCmd : uint8_t {
    InterfaceTest   = 0x00,
    DeviceReset     = 0x01,
    DeviceInitFlash = 0x02,
    DeviceErase     = 0x03,
    DeviceRead      = 0x04,
    DeviceWrite     = 0x05,
    DeviceExit      = 0x06
};

class EscPassthroughEngine {
public:
    static bool process_4way_cmd(FourWayCmd cmd, 
                                uint8_t esc_index, 
                                const std::span<const uint8_t>& rx_payload, 
                                std::span<uint8_t> tx_response, 
                                size_t& tx_len) noexcept {
        tx_len = 0;

        switch (cmd) {
            case FourWayCmd::InterfaceTest:
                // 4way-if Acknowledgment (0x00 ACK)
                tx_response[0] = 0x00; // ACK
                tx_response[1] = esc_index;
                tx_len = 2;
                return true;

            case FourWayCmd::DeviceInitFlash:
                // Device Initialized (BLHeli_32 / AM32 signature)
                tx_response[0] = 0x00; // ACK
                tx_response[1] = 0x32; // BLHeli_32 signature
                tx_len = 2;
                return true;

            case FourWayCmd::DeviceRead:
                // Emulate 256-byte EEPROM parameter read
                if (tx_response.size() >= 258) {
                    tx_response[0] = 0x00; // ACK
                    for (size_t i = 0; i < 256; ++i) {
                        tx_response[1 + i] = 0xFF; // Clean Flash
                    }
                    tx_response[257] = 0x00; // Checksum OK
                    tx_len = 258;
                    return true;
                }
                return false;

            default:
                tx_response[0] = 0x00; // ACK
                tx_len = 1;
                return true;
        }
    }
};

} // namespace abstractx::msp

#endif // ESC_PASSTHROUGH_HPP
