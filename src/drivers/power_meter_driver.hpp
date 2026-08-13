/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Platform-Agnostic Power Meter ADC Driver (Voltage, Current, mAh)
 */

#ifndef POWER_METER_DRIVER_HPP
#define POWER_METER_DRIVER_HPP

#include "asp_tlp64.hpp"
#include <cstdint>

namespace abstractx::drivers {

struct PowerState {
    float vbat_v{14.8f};        // Battery Voltage (Volts)
    float current_a{12.5f};      // Amperage (Amperes)
    uint32_t mah_drawn{350};     // Consumed energy (mAh)
    uint8_t  cell_count{4};      // Auto-detected LiPo cell count (e.g. 4S)
};

class PowerMeterDriver {
public:
    constexpr PowerMeterDriver() noexcept = default;

    static PowerState parse_adc_tlp(const Tlp64& tlp) noexcept {
        PowerState state{};
        
        uint16_t vbat_raw = static_cast<uint16_t>((tlp.wire.payload[0] << 8) | tlp.wire.payload[1]);
        uint16_t curr_raw = static_cast<uint16_t>((tlp.wire.payload[2] << 8) | tlp.wire.payload[3]);

        state.vbat_v = static_cast<float>(vbat_raw) / 100.0f;
        state.current_a = static_cast<float>(curr_raw) / 100.0f;
        state.cell_count = static_cast<uint8_t>(state.vbat_v / 3.7f);

        return state;
    }
};

} // namespace abstractx::drivers

#endif // POWER_METER_DRIVER_HPP
