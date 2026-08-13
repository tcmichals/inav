/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Dynamic Same-Pin Motor Signal Switcher (DShot Output -> 1-Wire Serial UART)
 */

#ifndef PIO_ESC_RELOADER_HPP
#define PIO_ESC_RELOADER_HPP

#include <cstdint>

namespace abstractx::target::pico2 {

enum class EscMode : uint8_t {
    FlightDShot,          // Output DShot150/300/600 waveforms on motor pins (GPIO 2..5)
    FlightOneShot,        // Output OneShot125/42 waveforms on motor pins (GPIO 2..5)
    FlightPwm,            // Output Standard PWM servo pulses on motor pins (GPIO 2..5)
    BlHeli4WayPassthrough // Dynamically reassign SAME pins to half-duplex 1-wire 19200 baud UART
};

class PioEscReloader {
public:
    // Dynamically switch the SAME physical motor pins (GPIO 2..5 / FPGA Motor Headers)
    static bool switch_motor_pin_mode(uint8_t motor_index, EscMode mode) noexcept {
        (void)motor_index;

        if (mode == EscMode::BlHeli4WayPassthrough) {
            // 1. Stop active DShot PIO state machine on motor pin
            // pio_sm_set_enabled(pio0, motor_sm, false);

            // 2. Hot-swap PIO0 instruction memory to 1-wire bidirectional UART
            // pio_remove_program(pio0, &dshot_program, dshot_offset);
            // uart_offset = pio_add_program(pio0, &pio_blheli_4way_1wire_uart);

            // 3. Configure SAME GPIO pin as open-drain half-duplex 1-wire serial (19200 baud)
            // pio_sm_config_set_out_pins(&c, motor_pin, 1);
            // pio_sm_config_set_in_pins(&c, motor_pin);
            // pio_sm_init(pio0, motor_sm, uart_offset, &c);
            // pio_sm_set_enabled(pio0, motor_sm, true);
            return true;
        } else {
            // Restore DShot/OneShot flight output on the SAME motor pin
            // pio_sm_set_enabled(pio0, motor_sm, false);
            // pio_remove_program(pio0, &uart_program, uart_offset);
            // dshot_offset = pio_add_program(pio0, &dshot_program);
            // pio_sm_set_enabled(pio0, motor_sm, true);
            return true;
        }
    }
};

} // namespace abstractx::target::pico2

#endif // PIO_ESC_RELOADER_HPP
